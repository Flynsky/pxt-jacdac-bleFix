#!/usr/bin/env python3
"""
calliope_uart.py - connect to a Calliope mini over Bluetooth (BLE UART) and print what it sends.

- finds the Calliope automatically (or pass --address)
- handles pairing: tells you when the Calliope must be in pairing mode and pairs via BlueZ
- reconnects automatically when the connection drops
- anything you type + Enter is sent to the Calliope (bluetooth.onUartDataReceived)

Usage:
    .venv/bin/python calliope_uart.py                 # auto-detect
    .venv/bin/python calliope_uart.py -a CF:E7:78:91:66:AA --hex
"""
import argparse
import asyncio
import re
import sys
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

UART_TX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Calliope -> PC (indicate)
UART_RX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> Calliope (write)
NAME_RE = re.compile(r"^(Calliope mini|BBC micro:bit)")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]|[\x01\x02]")
FAILS_BEFORE_PAIRING = 3  # failed connects in a row -> assume pairing is needed

PAIR_MODE_HELP = "put the Calliope into PAIRING MODE: hold A+B, press and release RESET, then release A+B"
RESET_HELP = "press RESET on the Calliope (without A+B) so your program runs"


# ---------------------------------------------------------------- output

def _ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


_last_status = None


def status(msg, once=False):
    """Print a status line. once=True suppresses repeats of the same message."""
    global _last_status
    if once and msg == _last_status:
        return
    _last_status = msg
    print(f"\033[2m{_ts()}\033[0m \033[1;33m» {msg}\033[0m", flush=True)


def error(msg):
    global _last_status
    _last_status = None
    print(f"\033[2m{_ts()}\033[0m \033[1;31m✗ {msg}\033[0m", flush=True)


def received(msg):
    global _last_status
    _last_status = None
    print(f"\033[2m{_ts()}\033[0m \033[1;32m←\033[0m {msg}", flush=True)


class Stats:
    """--stats mode: counts throughput and lost "oi <n>" lines instead of printing them."""

    def __init__(self):
        self.t_window = time.monotonic()
        self.win_bytes = self.win_lines = 0
        self.total_lines = self.total_lost = 0
        self.last_counter = None

    def add_bytes(self, n):
        self.win_bytes += n

    def line(self, text):
        parts = text.split()
        if len(parts) < 2 or parts[0] != "oi" or not parts[1].isdigit():
            received(text)  # Jacdac/button events etc. are still printed
            return
        n = int(parts[1])
        if self.last_counter is not None:
            if n > self.last_counter + 1:
                self.total_lost += n - self.last_counter - 1
            elif n <= self.last_counter:
                status(f"counter restarted at {n} (was {self.last_counter}): Calliope rebooted?")
        self.last_counter = n
        self.win_lines += 1
        self.total_lines += 1

    def tick(self):
        dt = time.monotonic() - self.t_window
        if dt < 1.0:
            return
        bps = self.win_bytes / dt
        status(f"{self.win_lines / dt:5.1f} lines/s  {bps:7.0f} B/s ({bps * 8 / 1000:5.1f} kbit/s)  "
               f"total {self.total_lines} lines, lost {self.total_lost}")
        self.t_window = time.monotonic()
        self.win_bytes = self.win_lines = 0


class LinePrinter:
    """Collects UART chunks (max ~20 bytes each) and prints whole lines."""

    def __init__(self, hex_mode, on_line=received):
        self.hex_mode = hex_mode
        self.on_line = on_line
        self.buf = b""
        self.last = 0.0

    def feed(self, chunk: bytearray):
        if self.hex_mode:
            received(f"{bytes(chunk).hex(' ')}   {bytes(chunk).decode(errors='replace')!r}")
            return
        self.buf += bytes(chunk)
        self.last = time.monotonic()
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            self.on_line(line.rstrip(b"\r").decode(errors="replace"))

    def flush(self, idle=0.0):
        """Print a partial line (no newline) once nothing new arrived for `idle` seconds."""
        if self.buf and time.monotonic() - self.last >= idle:
            self.on_line(self.buf.decode(errors="replace"))
            self.buf = b""


# ---------------------------------------------------------------- BlueZ helpers (bluetoothctl)

async def btctl(*args, timeout=10.0) -> str:
    proc = await asyncio.create_subprocess_exec(
        "bluetoothctl", *args,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        return ""
    return ANSI_RE.sub("", out.decode(errors="replace"))


async def bt_info(addr) -> dict:
    info = {}
    for line in (await btctl("info", addr)).splitlines():
        m = re.match(r"\s+(\w+): (.*)", line)
        if m:
            info[m.group(1)] = m.group(2).strip()
    return info


async def _rfkill_soft_blocked() -> bool:
    try:
        proc = await asyncio.create_subprocess_exec(
            "rfkill", "list", "bluetooth",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        out, _ = await proc.communicate()
    except FileNotFoundError:
        return False
    return "Soft blocked: yes" in out.decode(errors="replace")


async def ensure_powered():
    """Make sure the adapter is on; waits (instead of failing) while Bluetooth is disabled."""
    while "Powered: yes" not in await btctl("show"):
        if await _rfkill_soft_blocked():
            status("Bluetooth is switched off in the system. Turn it on in the settings "
                   "(or run: rfkill unblock bluetooth). Waiting ...", once=True)
        else:
            status("Bluetooth adapter is off, turning it on", once=True)
            await btctl("power", "on")
        await asyncio.sleep(2)


async def bt_pair(addr, timeout=30.0) -> bool:
    """Pair with Just Works via an interactive bluetoothctl (it provides the pairing agent)."""
    proc = await asyncio.create_subprocess_exec(
        "bluetoothctl",
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT)

    def send(cmd):
        proc.stdin.write((cmd + "\n").encode())

    loop = asyncio.get_running_loop()
    send("agent NoInputNoOutput")
    send("default-agent")
    send("scan le")  # make sure BlueZ currently knows the device
    await asyncio.sleep(2)
    send("scan off")
    send(f"pair {addr}")
    await proc.stdin.drain()

    ok, reason = False, "timeout"
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        try:
            raw = await asyncio.wait_for(proc.stdout.readline(), deadline - loop.time())
        except asyncio.TimeoutError:
            break
        if not raw:
            break
        line = ANSI_RE.sub("", raw.decode(errors="replace")).strip()
        if "Pairing successful" in line or "AlreadyExists" in line:
            ok = True
            break
        if "Failed to pair" in line or "not available" in line:
            reason = line.split("#")[-1].strip()
            break
        if "Passkey" in line or "Confirm" in line:
            status(f"bluetoothctl: {line}")

    send(f"disconnect {addr}")
    send("quit")
    try:
        await proc.stdin.drain()
        await asyncio.wait_for(proc.wait(), 5)
    except (asyncio.TimeoutError, ConnectionResetError, BrokenPipeError):
        proc.kill()
    if not ok:
        error(f"Pairing failed ({reason})")
    return ok


# ---------------------------------------------------------------- BLE

async def scan_for_calliope(address, timeout=8.0):
    """Return (BLEDevice, AdvertisementData) of the strongest matching Calliope, or None."""
    loop = asyncio.get_running_loop()
    best = {}
    found = asyncio.Event()

    def on_adv(dev, adv):
        if address:
            match = dev.address.upper() == address
        else:
            match = bool(NAME_RE.match(adv.local_name or ""))
        if match and (not best or adv.rssi > best["adv"].rssi):
            best.update(dev=dev, adv=adv)
            loop.call_later(1.0, found.set)  # short grace period for a stronger one

    async with BleakScanner(on_adv):
        try:
            await asyncio.wait_for(found.wait(), timeout)
        except asyncio.TimeoutError:
            pass
    return (best["dev"], best["adv"]) if best else None


async def session(dev, hex_mode, stats=None):
    """Connect, subscribe to UART and print until disconnected.

    Returns "fail" (couldn't connect / dropped right away), "no_uart" or "dropped".
    """
    disconnected = asyncio.Event()
    client = BleakClient(dev, disconnected_callback=lambda _c: disconnected.set(), timeout=10)
    loop = asyncio.get_running_loop()
    printer = LinePrinter(hex_mode, on_line=stats.line if stats else received)
    t0 = time.monotonic()
    stdin_reader = False
    active = [True]  # cleared when this session ends, see on_data

    try:
        await client.connect()
    except (BleakError, asyncio.TimeoutError, TimeoutError, EOFError) as e:
        status(f"connect failed: {e or type(e).__name__}")
        return "fail"

    try:
        tx = client.services.get_characteristic(UART_TX)
        if tx is None:
            return "no_uart"
        # Only the handler of the current session may print, in case BlueZ/bleak still
        # delivers a late notification to the handler of an earlier, disconnected session.
        def on_data(_c, d):
            if active[0]:
                if stats:
                    stats.add_bytes(len(d))
                printer.feed(d)
        # Note: if a second program (another calliope_uart.py, ble-serial, ...) is connected
        # to the same Calliope at the same time, BlueZ delivers every value twice.
        await client.start_notify(tx, on_data)
        status(f"Connected to {dev.address} ✓ waiting for data (type a line + Enter to send it)")

        def on_stdin():
            line = sys.stdin.readline()
            if not line:  # EOF
                loop.remove_reader(sys.stdin)
                return
            payload = line.rstrip("\n").encode() + b"\n"

            async def write():
                try:
                    for i in range(0, len(payload), 20):
                        await client.write_gatt_char(UART_RX, payload[i:i + 20], response=False)
                except BleakError as e:
                    error(f"send failed: {e}")
            loop.create_task(write())

        try:
            loop.add_reader(sys.stdin, on_stdin)
            stdin_reader = True
        except (ValueError, OSError, NotImplementedError):
            pass

        while not disconnected.is_set():
            try:
                await asyncio.wait_for(disconnected.wait(), 0.1)
            except asyncio.TimeoutError:
                printer.flush(idle=0.3)
                if stats:
                    stats.tick()
        printer.flush()
        if time.monotonic() - t0 < 3:
            status("Calliope dropped the connection right away")
            return "fail"
        status("Disconnected")
        return "dropped"
    except (BleakError, asyncio.TimeoutError, TimeoutError, EOFError) as e:
        status(f"connection lost: {e or type(e).__name__}")
        return "fail" if time.monotonic() - t0 < 3 else "dropped"
    finally:
        active[0] = False
        if stdin_reader:
            loop.remove_reader(sys.stdin)
        if client.is_connected:
            try:
                await client.stop_notify(UART_TX)
            except (BleakError, ValueError):
                pass
        # Always call disconnect(), also after the Calliope dropped the link: only this closes
        # the client's D-Bus connection. Otherwise BlueZ keeps its notify session alive and
        # re-enables it on the next connection, so every value arrived twice (3x, ...).
        try:
            await client.disconnect()
        except (BleakError, asyncio.TimeoutError, TimeoutError):
            pass


# ---------------------------------------------------------------- main loop

async def run(args):
    await ensure_powered()
    address = args.address.upper() if args.address else None
    fails = 0
    needs_pairing = False
    just_paired = False  # paired in this pairing-mode session, waiting for the user to reset
    stats = Stats() if args.stats else None  # kept across reconnects: totals cover the whole run

    status(f"Looking for {'Calliope ' + address if address else 'a Calliope mini'} ...")
    while True:
        try:
            found = await scan_for_calliope(address)
        except BleakError as e:
            error(f"scan failed: {e}")
            await ensure_powered()
            await asyncio.sleep(2)
            continue

        if not found:
            if address and (await bt_info(address)).get("Connected") == "yes":
                status("Calliope is held by another BlueZ connection, disconnecting it")
                await btctl("disconnect", address)
                continue
            status("No Calliope found. Is it on and running a program with Bluetooth? "
                   f"(or {PAIR_MODE_HELP})", once=True)
            continue

        dev, adv = found
        name = adv.local_name or dev.name or "?"
        if address is None:
            address = dev.address.upper()
            status(f"Found '{name}' at {address} (RSSI {adv.rssi})")
        paired = (await bt_info(address)).get("Paired") == "yes"

        # Always try to connect first. The advertised name can't tell pairing mode apart:
        # with "No Pairing Required" the Calliope shows "[xxxxx]" in normal mode too.
        # Pairing mode is recognized by the missing UART service instead.
        result = await session(dev, args.hex, stats)

        if result == "dropped":
            fails, needs_pairing, just_paired = 0, False, False
        elif result == "no_uart":
            fails = 0
            if just_paired:
                status(f"Still in pairing mode. Now {RESET_HELP}.", once=True)
            else:
                # Pairing mode (or a program without UART). Flashing wipes the Calliope's
                # bonds, so a bond on the PC side can't be trusted: pair fresh once.
                status("No UART service: Calliope is in pairing mode (or the program has no "
                       "bluetooth.startUartService()). Pairing ...")
                if paired:
                    await btctl("remove", address)
                if await bt_pair(address):
                    needs_pairing, just_paired = False, True
                    status(f"Paired ✓  Now {RESET_HELP}.")
                else:
                    status(f"Try again: {PAIR_MODE_HELP}.")
            await asyncio.sleep(2)
        else:
            fails += 1
            if fails >= FAILS_BEFORE_PAIRING:
                fails = 0
                if paired:
                    status("Stored pairing seems outdated (Calliope reflashed?), removing it")
                    await btctl("remove", address)
                    paired = False
                else:
                    needs_pairing = True
                if needs_pairing:
                    status(f"Pairing needed: {PAIR_MODE_HELP}.", once=True)
            await asyncio.sleep(1)


def main():
    p = argparse.ArgumentParser(description="Print everything a Calliope mini sends over Bluetooth UART.")
    p.add_argument("-a", "--address", help="Calliope MAC address (default: first Calliope found)")
    p.add_argument("--hex", action="store_true", help="print raw chunks as hex")
    p.add_argument("--stats", action="store_true",
                   help="stress test: print throughput and lost 'oi <n>' lines once per second "
                        "instead of every line (other lines are still printed)")
    args = p.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print()
        status("bye")


if __name__ == "__main__":
    main()
