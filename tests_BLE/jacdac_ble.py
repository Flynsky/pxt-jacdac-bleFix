#!/usr/bin/env python3
"""
jacdac_ble.py - receive Jacdac frames from a Calliope mini over Bluetooth LE and print them decoded.

Talks to the Jacdac GATT service of the bluetooth-transport extension (JacdacBLE.cpp) without a
browser, to debug the firmware side of the Jacdac-dashboard-over-BLE work.

- finds the Calliope automatically (or pass --address), pairs via BlueZ when needed and
  reconnects when the link drops (same approach as calliope_uart.py, whose helpers it uses)
- reassembles the 20-byte BLE chunks into Jacdac frames, checks the CRC and decodes every packet
  (announce, registers, events, commands); service/register/event names come from the
  */constants.ts files of this repository
- prints a one-line summary of the Jacdac devices seen, once per second
- --identify sends the control "identify" command to the Calliope (tests PC -> Calliope)

Usage:
    python3 jacdac_ble.py                     # auto-detect, print every frame + summary
    python3 jacdac_ble.py -q                  # summary only
    python3 jacdac_ble.py --identify          # make the Calliope show its identify animation
    python3 jacdac_ble.py --raw --diag        # hex dump of chunks/frames + firmware diagnostics
    python3 jacdac_ble.py --selftest          # offline check of framing/CRC/decoder, no device
"""
import argparse
import asyncio
import glob
import os
import re
import struct
import sys
import time
from collections import namedtuple

# ---------------------------------------------------------------- protocol constants

JD_SERVICE = "f8530001-a97f-49f5-a554-3e373fbea2d5"
JD_TX = "f8530002-a97f-49f5-a554-3e373fbea2d5"    # Calliope -> PC (notify/indicate); "RX" in jacdac-ts
JD_RX = "f8530003-a97f-49f5-a554-3e373fbea2d5"    # PC -> Calliope (write without response)
JD_DIAG = "f8530004-a97f-49f5-a554-3e373fbea2d5"  # jd_diagnostics_t (read)

# BLE chunking (JacdacBLE.cpp / jacdac-ts bluetooth.ts): 20-byte writes = 2 header bytes + 18 data.
# byte0 = total chunk count & 0x7f, | 0x80 on the first chunk; byte1 = chunks still to come after
# this one (total-1 ... 0).
CHUNK_DATA = 18
FIRST_CHUNK = 0x80
MAX_CHUNKS = 0x7f

# Jacdac frame (jdprotocol.h): crc u16, size u8 (bytes after the 12-byte header), flags u8,
# device id 8 bytes; then packets: service_size u8, service_index u8, service_command u16, data,
# each padded to 4 bytes. The CRC covers everything after the crc field.
HEADER = 12
MAX_FRAME = 252  # JD_SERIAL_PAYLOAD_SIZE + 16

FLAG_COMMAND = 0x01            # device id is the recipient (else the sender)
FLAG_ACK_REQUESTED = 0x02
FLAG_IS_SERVICE_CLASS = 0x04   # device id holds a service class (broadcast)
FLAG_LOOPBACK = 0x40
FLAG_VNEXT = 0x80

SIDX_PIPE = 0x3e
SIDX_CRC_ACK = 0x3f

CMD_GET_REG = 0x1000
CMD_SET_REG = 0x2000
CMD_TYPE_MASK = 0xf000
CMD_REG_MASK = 0x0fff
CMD_EVENT_MASK = 0x8000
CMD_EVENT_CODE_MASK = 0xff
CMD_EVENT_COUNTER_POS = 8
CMD_EVENT_COUNTER_MASK = 0x7f

CMD_ANNOUNCE = 0x00
CTRL_IDENTIFY = 0x81

SRV_CONTROL = 0x00000000
SRV_UNIQUE_BRAIN = 0x103c4ee5

ANNOUNCE_RESTART_MASK = 0x0f
ANNOUNCE_IS_CLIENT = 0x800
ANNOUNCE_FLAG_NAMES = ((0x0100, "ack"), (0x0200, "broadcast"), (0x0400, "frames"),
                       (0x0800, "client"), (0x1000, "reliable-cmds"))

DIAG_FIELDS = ("bus_state", "bus_lo_error", "bus_uart_error", "bus_timeout_error",
               "packets_sent", "packets_received", "packets_dropped")  # jdlow.h jd_diagnostics_t

SUMMARY_INTERVAL = 1.0


# ---------------------------------------------------------------- CRC, hashes, framing

def crc16(data) -> int:
    """jd_crc16() from jdutil.c (CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff)."""
    crc = 0xFFFF
    for b in data:
        x = ((crc >> 8) ^ b) & 0xFF
        x ^= x >> 4
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF
    return crc


def fnv1(data) -> int:
    """hash_fnv1() from pxt core.cpp (FNV-1, 32 bit)."""
    h = 0x811C9DC5
    for b in data:
        h = ((h * 0x1000193) & 0xFFFFFFFF) ^ b
    return h


def short_id(device_id: bytes) -> str:
    """jacdac.shortDeviceId() from routing.ts: the 4-character id the dashboard shows (e.g. "VR12")."""
    h = fnv1(device_id)
    h = (h ^ (h >> 30)) & ((1 << 30) - 1)  # Buffer.hash(30)
    return (chr(0x41 + h % 26) + chr(0x41 + (h // 26) % 26) +
            chr(0x30 + (h // (26 * 26)) % 10) + chr(0x30 + (h // (26 * 26 * 10)) % 10))


Packet = namedtuple("Packet", "index command payload")
Frame = namedtuple("Frame", "crc crc_ok size flags device_id packets extra error")


class FrameError(Exception):
    pass


def parse_frame(buf) -> Frame:
    """Split a reassembled frame into its packets. Raises FrameError if the header is unusable."""
    buf = bytes(buf)
    if len(buf) < HEADER + 4:
        raise FrameError(f"frame too short ({len(buf)} bytes)")
    crc, size, flags = struct.unpack_from("<HBB", buf)
    total = HEADER + size
    if total > len(buf):
        raise FrameError(f"size byte says {total} bytes, only {len(buf)} received")
    body = buf[HEADER:total]
    packets, ptr, error = [], 0, None
    while ptr < size:
        if ptr + 4 > size:
            error = f"{size - ptr} stray bytes at the end"
            break
        n, index, command = struct.unpack_from("<BBH", body, ptr)
        if ptr + 4 + n > size:
            error = f"packet at offset {ptr} overruns the frame ({n} data bytes)"
            break
        packets.append(Packet(index, command, body[ptr + 4:ptr + 4 + n]))
        ptr += (4 + n + 3) & ~3
    if not packets and error is None:
        error = "no packets"
    return Frame(crc, crc16(buf[2:total]) == crc, size, flags, buf[4:12], packets,
                 len(buf) - total, error)


def build_frame(device_id: bytes, packets, flags=0) -> bytes:
    """Build a frame from (service_index, service_command, payload) tuples, CRC included."""
    body = b""
    for index, command, payload in packets:
        p = struct.pack("<BBH", len(payload), index, command) + bytes(payload)
        body += p + b"\0" * (-len(p) % 4)
    if HEADER + len(body) > MAX_FRAME:
        raise FrameError(f"frame too large ({HEADER + len(body)} bytes)")
    rest = struct.pack("<BB", len(body), flags) + bytes(device_id) + body
    return struct.pack("<H", crc16(rest)) + rest


def identify_frame(device_id: bytes) -> bytes:
    """Command for the device's control service (index 0): identify (0x81), no payload."""
    return build_frame(device_id, [(0, CTRL_IDENTIFY, b"")], FLAG_COMMAND)


def chunk_frame(frame: bytes):
    """Split a frame into the 20-byte BLE writes the firmware expects (see JacdacBLE.cpp)."""
    total = max(1, -(-len(frame) // CHUNK_DATA))
    if total > MAX_CHUNKS:
        raise FrameError(f"frame needs {total} chunks (max {MAX_CHUNKS})")
    return [bytes([(total & 0x7F) | (FIRST_CHUNK if i == 0 else 0), total - 1 - i])
            + frame[i * CHUNK_DATA:(i + 1) * CHUNK_DATA] for i in range(total)]


class Reassembler:
    """Collects BLE chunks into frames. A frame with a missing or out-of-order chunk is dropped
    (never delivered partially); chunks that don't belong to a started frame are ignored."""

    def __init__(self):
        self.buf = None
        self.expected = 0
        self.out_of_order = 0   # frames dropped because a chunk was missing / out of order
        self.incomplete = 0     # frames dropped because a new frame started before they finished
        self.stray = 0          # non-first chunks without a started frame
        self.bad = 0            # chunks shorter than the header or claiming 0 chunks

    def reset(self):
        self.buf = None

    def feed(self, chunk):
        """Add one chunk; returns the complete frame (bytes) or None."""
        chunk = bytes(chunk)
        if len(chunk) < 2:
            self.bad += 1
            self.buf = None
            return None
        head, remaining = chunk[0], chunk[1]
        if head & FIRST_CHUNK:
            if self.buf is not None:
                self.incomplete += 1
            total = head & MAX_CHUNKS
            if total == 0:
                self.bad += 1
                self.buf = None
                return None
            self.buf, self.expected = bytearray(), total - 1
        elif self.buf is None:
            self.stray += 1
            return None
        else:
            self.expected -= 1
        if remaining != self.expected:
            self.out_of_order += 1
            self.buf = None
            return None
        self.buf += chunk[2:]
        if self.expected == 0:
            frame, self.buf = bytes(self.buf), None
            return frame
        return None


# ---------------------------------------------------------------- names from */constants.ts

SRV_RE = re.compile(r"export const SRV_(\w+)\s*=\s*(0x[0-9a-fA-F]+|\d+)")
ENUM_RE = re.compile(r"export const enum (\w+)\s*\{(.*?)\n\s*\}", re.S)
MEMBER_RE = re.compile(r"^\s*(\w+)\s*=\s*(0x[0-9a-fA-F]+|\d+)\s*,?\s*(?://.*)?$", re.M)
ENUM_KIND_RE = re.compile(r"^(\w*?)(Reg|Cmd|Event)$")


def _num(text):
    return int(text, 16) if text.lower().startswith("0x") else int(text)


class Registry:
    """Service, register, command and event names parsed from the repo's */constants.ts."""

    def __init__(self, root=None):
        self.services = {}                                  # class -> "rotary_encoder"
        self.enums = {}                                     # (class, kind) -> {value: name}
        self.system = {"Reg": {}, "Cmd": {}, "Event": {}}   # common to all services
        if root is None:
            root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for path in sorted(glob.glob(os.path.join(root, "*", "constants.ts"))):
            try:
                with open(path, encoding="utf-8", errors="replace") as f:
                    self._load(f.read())
            except OSError:
                pass

    def _load(self, text):
        classes = []
        for name, value in SRV_RE.findall(text):
            classes.append(_num(value))
            self.services.setdefault(_num(value), name.lower())
        for enum_name, body in ENUM_RE.findall(text):
            m = ENUM_KIND_RE.match(enum_name)
            if not m:
                continue
            members = {_num(v): k for k, v in MEMBER_RE.findall(body)}
            if m.group(1) == "System" and not classes:
                self.system[m.group(2)].update(members)
            for cls in classes:
                self.enums.setdefault((cls, m.group(2)), {}).update(members)

    def service(self, cls):
        if cls is None:
            return "?"
        return self.services.get(cls, f"0x{cls:08x}")

    def name(self, cls, kind, value):
        found = self.enums.get((cls, kind), {}).get(value) if cls is not None else None
        return found or self.system[kind].get(value)


# ---------------------------------------------------------------- decoding / device tracking

def fmt_payload(payload: bytes) -> str:
    if not payload:
        return ""
    text = payload.hex(" ")
    if len(payload) in (1, 2, 4):
        text += f" ({int.from_bytes(payload, 'little')})"
    return text


def parse_announce(payload: bytes):
    """Control announce report: u16 flags, u8 packet count, u8 reserved, u32 service classes[]."""
    if len(payload) < 4:
        return None
    flags, packet_count = struct.unpack_from("<HB", payload)
    classes = [c for (c,) in struct.iter_unpack("<I", payload[4:4 + (len(payload) - 4) // 4 * 4])]
    return flags, packet_count, classes


class Device:
    def __init__(self, device_id):
        self.id = device_id
        self.short = short_id(device_id)
        self.services = [SRV_CONTROL]   # service class per index, from the announce
        self.announce_flags = None
        self.first_seen = self.last_seen = time.monotonic()
        self.frames = 0

    @property
    def is_client(self):
        return self.announce_flags is not None and bool(self.announce_flags & ANNOUNCE_IS_CLIENT)

    def service_class(self, index):
        return self.services[index] if index < len(self.services) else None


class Monitor:
    """Turns BLE chunks into decoded output and keeps track of the devices on the bus."""

    def __init__(self, registry, out=print, quiet=False, raw=False):
        self.reg = registry
        self.out = out          # gets one text block per frame
        self.quiet = quiet
        self.raw = raw
        self.reasm = Reassembler()
        self.devices = {}       # device id bytes -> Device
        self.services_seen = [] # GATT service UUIDs of a Calliope without Jacdac service
        self.frames = self.crc_errors = self.bad_frames = 0
        self.win_frames = self.win_bytes = 0
        self.t_window = self.last_frame = time.monotonic()

    # ---- input

    def on_chunk(self, chunk):
        self.win_bytes += len(chunk)
        if self.raw:
            self.out(f"chunk {bytes(chunk).hex(' ')}")
        frame = self.reasm.feed(chunk)
        if frame is not None:
            self.on_frame(frame)

    def on_frame(self, buf):
        now = time.monotonic()
        self.frames += 1
        self.win_frames += 1
        self.last_frame = now
        try:
            frame = parse_frame(buf)
        except FrameError as e:
            self.bad_frames += 1
            self.out(f"bad frame: {e}   {bytes(buf).hex(' ')}")
            return None
        if not frame.crc_ok:
            self.crc_errors += 1
        self._track(frame, now)
        if self.raw:
            self.out(f"frame {bytes(buf).hex(' ')}")
        if not self.quiet or not frame.crc_ok or frame.error:
            self.out(self.describe(frame))
        return frame

    # ---- device tracking

    def _track(self, frame, now):
        if frame.flags & (FLAG_COMMAND | FLAG_IS_SERVICE_CLASS):
            return  # device id is a recipient / service class, not the sender
        dev = self.devices.get(frame.device_id)
        if dev is None:
            dev = self.devices[frame.device_id] = Device(frame.device_id)
        dev.last_seen = now
        dev.frames += 1
        for p in frame.packets:
            if p.index == 0 and p.command == CMD_ANNOUNCE:
                ann = parse_announce(p.payload)
                if ann:
                    flags, _count, classes = ann
                    if (dev.announce_flags is not None and
                            (flags & ANNOUNCE_RESTART_MASK) < (dev.announce_flags & ANNOUNCE_RESTART_MASK)):
                        self.out(f"   device {dev.short} restarted")
                    dev.announce_flags = flags
                    dev.services = [SRV_CONTROL] + classes

    def brain(self):
        """The device that announces itself as a client (the Calliope running the MakeCode
        program); prefers one hosting the unique-brain service."""
        clients = [d for d in self.devices.values() if d.is_client]
        clients.sort(key=lambda d: (SRV_UNIQUE_BRAIN not in d.services, d.first_seen))
        return clients[0] if clients else None

    def find_device(self, spec):
        """Device by 16-hex-digit id or 4-character short id (case-insensitive)."""
        spec = spec.strip().lower()
        for dev in self.devices.values():
            if spec in (dev.id.hex(), dev.short.lower()):
                return dev
        return None

    # ---- output

    def describe(self, frame):
        """Frame as text: sender (reports) or "→ recipient" (commands), then its packets."""
        dev = None
        if frame.flags & FLAG_IS_SERVICE_CLASS:
            cls = int.from_bytes(frame.device_id[:4], "little")
            who = f"(to all {self.reg.service(cls)})"
        else:
            dev = self.devices.get(frame.device_id)
            short = dev.short if dev else short_id(frame.device_id)
            who = f"{'→ ' if frame.flags & FLAG_COMMAND else ''}{short} {frame.device_id.hex()}"
        notes = []
        if frame.flags & FLAG_ACK_REQUESTED:
            notes.append("ack-requested")
        if frame.flags & FLAG_VNEXT:
            notes.append("vnext")
        if not frame.crc_ok:
            notes.append(f"CRC MISMATCH (got 0x{frame.crc:04x}, "
                         f"expected 0x{crc16(self._frame_bytes(frame)):04x})")
        if frame.extra:
            notes.append(f"+{frame.extra} bytes after the frame")
        if frame.error:
            notes.append(f"BAD: {frame.error}")
        head = who + (f"  [{', '.join(notes)}]" if notes else "")
        lines = [self.describe_packet(frame, p, dev) for p in frame.packets]
        if len(lines) == 1:
            return f"{head}  {lines[0]}"
        return "\n".join([head] + [f"      {line}" for line in lines])

    @staticmethod
    def _frame_bytes(frame):
        body = b"".join(struct.pack("<BBH", len(p.payload), p.index, p.command) + p.payload +
                        b"\0" * (-(4 + len(p.payload)) % 4) for p in frame.packets)
        return struct.pack("<BB", frame.size, frame.flags) + frame.device_id + body

    def describe_packet(self, frame, p, dev):
        command = bool(frame.flags & FLAG_COMMAND)
        if p.index == SIDX_CRC_ACK:
            return f"#{p.index} crc-ack 0x{p.command:04x}"
        if p.index == SIDX_PIPE:
            return f"#{p.index} pipe 0x{p.command:04x} {fmt_payload(p.payload)}".rstrip()
        if frame.flags & FLAG_IS_SERVICE_CLASS:
            cls = int.from_bytes(frame.device_id[:4], "little")
        elif p.index == 0:
            cls = SRV_CONTROL
        else:
            cls = dev.service_class(p.index) if dev else None
        svc = self.reg.service(cls) if cls is not None else "?"
        prefix = f"#{p.index} {svc}"
        data = fmt_payload(p.payload)

        if p.index == 0 and p.command == CMD_ANNOUNCE and not command:
            ann = parse_announce(p.payload)
            if ann:
                flags, count, classes = ann
                names = [n for bit, n in ANNOUNCE_FLAG_NAMES if flags & bit]
                services = " ".join(f"{i}={self.reg.service(c)}" for i, c in enumerate(classes, 1))
                return (f"{prefix} announce restart={flags & ANNOUNCE_RESTART_MASK} "
                        f"[{','.join(names)}] pkts={count} services: {services or '-'}")
        if p.command & CMD_EVENT_MASK and not command:
            code = p.command & CMD_EVENT_CODE_MASK
            counter = (p.command >> CMD_EVENT_COUNTER_POS) & CMD_EVENT_COUNTER_MASK
            name = self.reg.name(cls, "Event", code) or f"0x{code:02x}"
            return f"{prefix} event {name} #{counter} {data}".rstrip()
        kind = p.command & CMD_TYPE_MASK
        if kind in (CMD_GET_REG, CMD_SET_REG):
            reg = p.command & CMD_REG_MASK
            name = self.reg.name(cls, "Reg", reg) or "reg"
            name = f"{name}(0x{reg:03x})"
            if kind == CMD_SET_REG:
                return f"{prefix} set {name} = {data}".rstrip()
            if command:
                return f"{prefix} get {name}"
            return f"{prefix} {name} = {data}".rstrip()
        name = self.reg.name(cls, "Cmd", p.command) or "cmd"
        return f"{prefix} {name}(0x{p.command:03x}) {data}".rstrip()

    def summary(self):
        """One status line; resets the per-second counters."""
        now = time.monotonic()
        dt = max(now - self.t_window, 1e-6)
        brain = self.brain()
        devs = []
        for dev in sorted(self.devices.values(), key=lambda d: d.first_seen):
            names = [self.reg.service(c) for c in dev.services[1:]]
            tag = "*" if dev is brain else ""
            devs.append(f"{dev.short}{tag}({','.join(names) or '-'}) {now - dev.last_seen:.1f}s")
        r = self.reasm
        drops = r.out_of_order + r.incomplete + r.stray + r.bad
        idle = now - self.last_frame
        text = (f"{len(self.devices)} dev: {'  '.join(devs) or '-'} | "
                f"{self.win_frames / dt:4.1f} frames/s {self.win_bytes / dt:5.0f} B/s | "
                f"total {self.frames}, crc err {self.crc_errors}, bad {self.bad_frames}, "
                f"chunk drops {drops}")
        if drops:
            text += f" (order {r.out_of_order}, incomplete {r.incomplete}, stray {r.stray}, bad {r.bad})"
        if idle > 2 * SUMMARY_INTERVAL:
            text += f" | no frames for {idle:.0f}s"
        self.t_window = now
        self.win_frames = self.win_bytes = 0
        return text


# ---------------------------------------------------------------- BLE session

class Identifier:
    """Sends identify frames to the target device once it is known (--identify)."""

    def __init__(self, client, mon, spec):
        self.client = client
        self.mon = mon
        self.spec = spec      # "auto" or a device id / short id
        self.pending = True
        self.lock = asyncio.Lock()
        self.announced_wait = False

    def target(self):
        return self.mon.brain() if self.spec == "auto" else self.mon.find_device(self.spec)

    async def poll(self, cu):
        if not self.pending:
            return
        dev = self.target()
        if dev is None:
            if not self.announced_wait:
                what = "the Calliope's announce" if self.spec == "auto" else f"device {self.spec}"
                cu.status(f"--identify: waiting for {what} to learn its device id ...")
                self.announced_wait = True
            return
        self.pending = False
        await self.send(dev, cu)

    async def send(self, dev, cu):
        from bleak.exc import BleakError
        frame = identify_frame(dev.id)
        async with self.lock:
            try:
                for chunk in chunk_frame(frame):  # one after another: no parallel GATT writes
                    await self.client.write_gatt_char(JD_RX, chunk, response=False)
            except (BleakError, asyncio.TimeoutError, TimeoutError) as e:
                cu.error(f"identify: write failed: {e or type(e).__name__}")
                return
        cu.status(f"→ identify sent to {dev.short} ({dev.id.hex()}): the Calliope should play "
                  f"its identify animation. Press Enter to send it again.")
        if self.mon.raw:
            self.mon.out(f"sent {frame.hex(' ')}")


async def read_diag(client, cu):
    from bleak.exc import BleakError
    try:
        data = bytes(await client.read_gatt_char(JD_DIAG))
    except (BleakError, asyncio.TimeoutError, TimeoutError) as e:
        cu.error(f"diag read failed: {e or type(e).__name__}")
        return
    if len(data) >= 4 * len(DIAG_FIELDS):
        values = struct.unpack_from(f"<{len(DIAG_FIELDS)}I", data)
        cu.status("diag " + " ".join(f"{k.replace('bus_', '')}={v}" for k, v in zip(DIAG_FIELDS, values)))
    else:
        cu.status(f"diag ({len(data)} bytes): {data.hex(' ') or 'empty'}")


async def session(dev, mon, args, cu):
    """Connect, subscribe to the Jacdac TX characteristic and decode until disconnected.

    Returns "fail" (couldn't connect / dropped right away), "no_jacdac" or "dropped".
    """
    from bleak import BleakClient
    from bleak.exc import BleakError

    disconnected = asyncio.Event()
    client = BleakClient(dev, disconnected_callback=lambda _c: disconnected.set(), timeout=10)
    loop = asyncio.get_running_loop()
    t0 = time.monotonic()
    stdin_reader = False
    active = [True]  # cleared when this session ends, see on_data

    try:
        await client.connect()
    except (BleakError, asyncio.TimeoutError, TimeoutError, EOFError) as e:
        cu.status(f"connect failed: {e or type(e).__name__}")
        return "fail"

    try:
        tx = client.services.get_characteristic(JD_TX)
        if tx is None:
            mon.services_seen = [s.uuid for s in client.services]
            return "no_jacdac"
        mon.reasm.reset()  # a frame cut by the last disconnect must not be continued

        # Only the handler of the current session may decode: BlueZ/bleak can still deliver a
        # late notification to the handler of an earlier, disconnected session.
        def on_data(_c, d):
            if active[0]:
                mon.on_chunk(d)

        await client.start_notify(tx, on_data)  # works for notify and indicate
        cu.status(f"Connected to {dev.address} ✓ waiting for Jacdac frames")

        ident = Identifier(client, mon, args.identify) if args.identify else None
        if ident:
            def on_stdin():
                if not sys.stdin.readline():  # EOF
                    loop.remove_reader(sys.stdin)
                    return
                ident.pending = True
                ident.announced_wait = False
            try:
                loop.add_reader(sys.stdin, on_stdin)
                stdin_reader = True
            except (ValueError, OSError, NotImplementedError):
                pass

        next_summary = time.monotonic() + SUMMARY_INTERVAL
        while not disconnected.is_set():
            try:
                await asyncio.wait_for(disconnected.wait(), 0.1)
            except asyncio.TimeoutError:
                if ident:
                    await ident.poll(cu)
                if time.monotonic() >= next_summary:
                    next_summary += SUMMARY_INTERVAL
                    cu.status(mon.summary())
                    if args.diag:
                        await read_diag(client, cu)
        if time.monotonic() - t0 < 3:
            cu.status("Calliope dropped the connection right away")
            return "fail"
        cu.status("Disconnected")
        return "dropped"
    except (BleakError, asyncio.TimeoutError, TimeoutError, EOFError) as e:
        cu.status(f"connection lost: {e or type(e).__name__}")
        return "fail" if time.monotonic() - t0 < 3 else "dropped"
    finally:
        active[0] = False
        if stdin_reader:
            loop.remove_reader(sys.stdin)
        if client.is_connected:
            try:
                await client.stop_notify(JD_TX)
            except (BleakError, ValueError):
                pass
        # Always disconnect, also after the Calliope dropped the link: only this closes the
        # client's D-Bus connection (see calliope_uart.py: otherwise values arrive twice).
        try:
            await client.disconnect()
        except (BleakError, asyncio.TimeoutError, TimeoutError):
            pass


async def run(args):
    sys.dont_write_bytecode = True  # no __pycache__/ in the repo from importing calliope_uart
    import calliope_uart as cu  # scanning, BlueZ pairing helpers and the output format
    from bleak.exc import BleakError

    mon = Monitor(Registry(), out=cu.received, quiet=args.quiet, raw=args.raw)
    if not mon.reg.services:
        cu.status("no */constants.ts found next to tests_BLE/: printing numbers instead of names")

    await cu.ensure_powered()
    address = args.address.upper() if args.address else None
    fails = 0
    needs_pairing = False
    just_paired = False  # paired in this pairing-mode session, waiting for the user to reset
    # A Calliope we can connect to but that has no Jacdac service is usually just running a
    # program without the Jacdac BLE transport, which pairing can't fix. It is only treated as
    # pairing mode after we asked the user to use pairing mode, or with --pair.
    pairing_hinted = args.pair

    cu.status(f"Looking for {'Calliope ' + address if address else 'a Calliope mini'} ...")
    while True:
        try:
            found = await cu.scan_for_calliope(address)
        except BleakError as e:
            cu.error(f"scan failed: {e}")
            await cu.ensure_powered()
            await asyncio.sleep(2)
            continue

        if not found:
            if address and (await cu.bt_info(address)).get("Connected") == "yes":
                cu.status("Calliope is held by another BlueZ connection, disconnecting it")
                await cu.btctl("disconnect", address)
                continue
            cu.status("No Calliope found. Is it on and running a program with Jacdac over "
                      f"Bluetooth? (or {cu.PAIR_MODE_HELP})", once=True)
            pairing_hinted = True
            continue

        dev, adv = found
        name = adv.local_name or dev.name or "?"
        if address is None:
            address = dev.address.upper()
            cu.status(f"Found '{name}' at {address} (RSSI {adv.rssi})")
        paired = (await cu.bt_info(address)).get("Paired") == "yes"

        result = await session(dev, mon, args, cu)

        if result == "dropped":
            fails, needs_pairing, just_paired, pairing_hinted = 0, False, False, False
        elif result == "no_jacdac":
            fails = 0
            found_services = describe_services(mon.services_seen)
            if needs_pairing or pairing_hinted:
                # The user was asked to use pairing mode (or passed --pair): pair now. Flashing
                # wipes the Calliope's bonds, so a stored bond on the PC side is removed first.
                cu.status(f"No Jacdac service (found: {found_services}). Assuming pairing mode, "
                          "pairing ...")
                if paired:
                    await cu.btctl("remove", address)
                ok = await cu.bt_pair(address)
                needs_pairing = pairing_hinted = False
                if ok:
                    just_paired = True
                    cu.status(f"Paired ✓  Now {cu.RESET_HELP}.")
                else:
                    cu.status(f"So the Calliope is probably not in pairing mode. {NO_JACDAC_HELP}")
            elif just_paired:
                cu.status(f"No Jacdac service (found: {found_services}). If the Calliope still "
                          f"shows the pairing pattern, {cu.RESET_HELP}. Otherwise: {NO_JACDAC_HELP}",
                          once=True)
            else:
                cu.status(f"Connected, but the Calliope has no Jacdac service (found: "
                          f"{found_services}). {NO_JACDAC_HELP}", once=True)
            await asyncio.sleep(5)
        else:
            fails += 1
            if fails >= cu.FAILS_BEFORE_PAIRING:
                fails = 0
                if paired:
                    cu.status("Stored pairing seems outdated (Calliope reflashed?), removing it")
                    await cu.btctl("remove", address)
                    paired = False
                else:
                    needs_pairing = True
                if needs_pairing:
                    cu.status(f"Pairing needed: {cu.PAIR_MODE_HELP}.", once=True)
            await asyncio.sleep(1)


NO_JACDAC_HELP = ("Its program needs the jacdac bluetooth-transport extension and has to reach "
                  "jacdac.start() (or a Jacdac block). Waiting for a program with it "
                  "(flash it, or press RESET) ...")

KNOWN_GATT_SERVICES = {  # first 8 hex digits of the UUID
    "00001800": "generic access", "00001801": "generic attribute", "0000180a": "device info",
    "0000fe59": "Nordic DFU", "e95d93b0": "DFU control", "e97dd91d": "partial flashing",
    "6e400001": "UART", "b112f5e6": "HF2 console", "f8530001": "Jacdac",
    "e95d93af": "event", "e95d0753": "accelerometer", "e95df2d8": "magnetometer",
    "e95d9882": "button", "e95d127b": "IO pin", "e95dd91d": "LED", "e95d6100": "temperature",
}


def describe_services(uuids):
    names = [KNOWN_GATT_SERVICES.get(u[:8].lower(), u[:8]) for u in uuids or []]
    return ", ".join(names) or "none"


# ---------------------------------------------------------------- offline self-test

def _fw_tx_chunks(frame):
    """Python port of the chunking loop in JacdacBLE::send() (firmware -> PC)."""
    length = len(frame)
    total = length // CHUNK_DATA + (length % CHUNK_DATA != 0)
    remaining, sent, out = total - 1, 0, []
    while sent < length:
        n = min(CHUNK_DATA, length - sent)
        head = (total & 0x7F) | (FIRST_CHUNK if sent == 0 else 0)
        out.append(bytes([head, remaining]) + frame[sent:sent + n])
        sent += n
        remaining = 0 if remaining == 0 else remaining - 1
    return out


def _fw_rx(chunks):
    """Python port of the reassembly in JacdacBLE::onDataWritten() (PC -> firmware)."""
    rx, counter, frames = bytearray(), 0, []
    for d in chunks:
        if d[0] & FIRST_CHUNK:
            rx, counter = bytearray(), d[0] & 0x7F
        counter = 0 if counter == 0 else counter - 1
        if d[1] == counter:
            rx += d[2:]
        if counter == 0:
            frames.append(bytes(rx))
            rx = bytearray()
    return frames


def _crc16_bitwise(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def selftest() -> int:
    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        mark = "\033[1;32m✓\033[0m" if ok else "\033[1;31m✗\033[0m"
        print(f"{mark} {name}" + (f"   ({detail})" if detail and not ok else ""))

    dev_id = bytes.fromhex("a1b2c3d4e5f60718")
    srv_button = 0x1473a263

    # CRC / hashes. Known answers computed with jd_crc16() from jdutil.c and with the
    # TypeScript shortDeviceId()/Buffer.hash() algorithm.
    check("crc16 check value of '123456789' is 0x29b1 (CRC-16/CCITT-FALSE)",
          crc16(b"123456789") == 0x29B1, hex(crc16(b"123456789")))
    sample = bytes(range(256)) * 3
    check("crc16 matches a bitwise CRC-16/CCITT-FALSE over 768 bytes",
          crc16(sample) == _crc16_bitwise(sample))
    check("short id of a1b2c3d4e5f60718 matches jacdac.shortDeviceId()",
          short_id(dev_id) == SELFTEST_SHORT_ID, short_id(dev_id))

    # identify command frame, byte for byte
    ident = identify_frame(dev_id)
    check("identify frame is byte-identical to the C-built reference",
          ident.hex() == SELFTEST_IDENTIFY_HEX, ident.hex())
    f = parse_frame(ident)
    check("identify frame: CRC ok, command flag, 1 packet (index 0, cmd 0x81)",
          f.crc_ok and f.flags == FLAG_COMMAND and f.size == 4 and f.device_id == dev_id
          and f.packets == [Packet(0, CTRL_IDENTIFY, b"")])

    # announce of a brain + decoding with names from constants.ts
    announce_payload = struct.pack("<HBB", 0x0C01, 3, 0) + struct.pack("<II", SRV_UNIQUE_BRAIN, srv_button)
    announce = build_frame(dev_id, [(0, CMD_ANNOUNCE, announce_payload)])
    check("announce frame is byte-identical to the C-built reference",
          announce.hex() == SELFTEST_ANNOUNCE_HEX, announce.hex())
    lines = []
    mon = Monitor(Registry(), out=lines.append)
    mon.on_frame(announce)
    brain = mon.brain()
    check("announce: device tracked as brain with services [control, unique brain, button]",
          brain is not None and brain.id == dev_id
          and brain.services == [SRV_CONTROL, SRV_UNIQUE_BRAIN, srv_button])
    have_names = bool(mon.reg.services)
    check("constants.ts loaded (service names known)", have_names, "no */constants.ts found")
    if have_names:
        check("announce is printed with flags and service names",
              lines and "announce restart=1 [frames,client]" in lines[-1]
              and "1=unique_brain 2=button" in lines[-1], lines[-1] if lines else "no output")
        check("control identify command is named 'Identify'",
              mon.reg.name(SRV_CONTROL, "Cmd", CTRL_IDENTIFY) == "Identify")

    # multi-packet frame: register report on the button + event, padding in between
    multi = build_frame(dev_id, [(2, CMD_GET_REG | 0x101, b"\x00\x10"),
                                 (2, CMD_EVENT_MASK | (5 << CMD_EVENT_COUNTER_POS) | 0x01, b"")])
    f = parse_frame(multi)
    check("multi-packet frame: 2 packets, padding skipped, CRC ok",
          f.crc_ok and f.error is None and len(f.packets) == 2
          and f.packets[0] == Packet(2, 0x1101, b"\x00\x10") and f.packets[1].command == 0x8501)
    mon.on_frame(multi)
    if have_names:
        check("multi-packet frame printed as button register + event",
              "#2 button" in lines[-1] and "event Down #5" in lines[-1], lines[-1])

    # corrupted / truncated frames
    bad = bytearray(announce)
    bad[20] ^= 0x40
    f = parse_frame(bad)
    check("flipped payload bit is reported as CRC mismatch", not f.crc_ok)
    mon.on_frame(bad)
    check("CRC mismatch is counted and printed", mon.crc_errors == 1 and "CRC MISMATCH" in lines[-1])
    try:
        parse_frame(announce[:-4])
        check("truncated frame is rejected", False, "no FrameError")
    except FrameError:
        check("truncated frame is rejected", True)

    # BLE chunking, both directions, against ports of the firmware code
    # frames are always a multiple of 4 bytes; 18 data bytes per chunk: 1, 2 (18+2), 2 (18+18),
    # 3, 4, 6 and 14 chunks (largest frame)
    sizes = [16, 20, 36, 40, 56, 100, MAX_FRAME]
    frames = [build_frame(dev_id, [(1, 0x80, bytes(range(n - HEADER - 4)))]) for n in sizes]
    ok = [len(fr) for fr in frames] == sizes
    check(f"test frames have sizes {sizes}", ok, str([len(fr) for fr in frames]))
    ok = True
    for fr in frames:
        r = Reassembler()
        got = [x for x in (r.feed(c) for c in _fw_tx_chunks(fr)) if x is not None]
        ok &= got == [fr]
    check("frames chunked by the firmware's send() loop reassemble exactly", ok)
    ok = all(chunk_frame(fr) == _fw_tx_chunks(fr) for fr in frames)
    check("our chunker produces the same chunks as the firmware's send() loop", ok)
    ok = all(_fw_rx(chunk_frame(fr)) == [fr] for fr in frames)
    check("the firmware's receive logic reassembles our chunks exactly", ok)
    ok = all(len(c) <= 20 for fr in frames for c in chunk_frame(fr))
    check("no chunk is longer than 20 bytes (ATT MTU 23)", ok)

    # broken chunk streams
    big = frames[sizes.index(100)]
    chunks = chunk_frame(big)  # 6 chunks
    r = Reassembler()
    out = [r.feed(c) for c in [chunks[0], chunks[2], chunks[1]] + chunks]
    check("out-of-order chunk drops that frame, the next frame still arrives",
          [x for x in out if x] == [big] and r.out_of_order == 1)
    r = Reassembler()
    out = [r.feed(c) for c in chunks[1:] + chunks]
    check("stray chunks without a first chunk are ignored",
          [x for x in out if x] == [big] and r.stray == len(chunks) - 1)
    r = Reassembler()
    out = [r.feed(c) for c in chunks[:1] + chunks]
    check("a frame cut short by a new first chunk is dropped, not delivered",
          [x for x in out if x] == [big] and r.incomplete == 1)
    r = Reassembler()
    out = [r.feed(c) for c in [b"", b"\x80", bytes([0x80, 0])] + chunks]
    check("empty / header-less / zero-count chunks are ignored",
          [x for x in out if x] == [big] and r.bad == 3)

    failed = results.count(False)
    print(f"\n{len(results) - failed}/{len(results)} checks passed")
    return 1 if failed else 0


# Known answers for the self-test, computed independently of this file:
#  - identify frame / announce CRC: jd_crc16() compiled from jdutil.c over the same bytes
#  - short id: shortDeviceId() + Buffer.hash(30) + hash_fnv1 run in Node.js
SELFTEST_SHORT_ID = "RS40"
SELFTEST_IDENTIFY_HEX = "b8580401a1b2c3d4e5f6071800008100"
SELFTEST_ANNOUNCE_HEX = "53831000a1b2c3d4e5f607180c000000010c0300e54e3c1063a27314"


# ---------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(
        description="Receive Jacdac frames from a Calliope mini over Bluetooth LE "
                    "(bluetooth-transport extension) and print them decoded.",
        epilog="The Calliope must run a program with the jacdac bluetooth-transport extension. "
               "Pairing follows the project's Bluetooth setting: with pairing required, put the "
               "Calliope into pairing mode (A+B, RESET) when asked; the tool pairs via BlueZ. "
               "Stop with Ctrl+C.")
    p.add_argument("-a", "--address", help="Calliope MAC address (default: strongest Calliope found)")
    p.add_argument("-q", "--quiet", action="store_true",
                   help="print only the once-per-second summary (and broken frames), not every frame")
    p.add_argument("--raw", action="store_true",
                   help="also hex-dump every BLE chunk and every reassembled frame")
    p.add_argument("--identify", nargs="?", const="auto", metavar="DEVICE",
                   help="send the control 'identify' command (0x81) over BLE to DEVICE (16-hex-digit "
                        "device id or 4-character short id); default: the Calliope itself, found as "
                        "the device that announces itself as a client. Press Enter to send it again.")
    p.add_argument("--pair", action="store_true",
                   help="pair first: put the Calliope into pairing mode (A+B, RESET) before starting. "
                        "Without it, the tool only pairs after it asked you to use pairing mode.")
    p.add_argument("--diag", action="store_true",
                   help="read the firmware's diagnostics characteristic (f8530004) once per second")
    p.add_argument("--selftest", action="store_true",
                   help="check CRC, chunking and decoding offline (no Calliope needed), then exit")
    args = p.parse_args()

    if args.selftest:
        sys.exit(selftest())
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
