# jacdac-bluetooth-transport

Sends the Jacdac traffic of a micro:bit V2 / Calliope mini 3 over Bluetooth LE, so the
[Jacdac dashboard](https://jacdac.github.io/jacdac-docs/dashboard/) can connect without a cable
(Web Bluetooth, Chrome/Edge). The USB connection (WebUSB bridge) keeps working at the same time.

In MakeCode: Extensions → paste `https://github.com/Flynsky/pxt-jacdac-bleFix/bluetooth-transport`.

## How it works

The device exposes a GATT service `f8530001-a97f-49f5-a554-3e373fbea2d5`:

| Characteristic | Direction | Properties |
|---|---|---|
| `f8530002-…` TX | device → PC | notify |
| `f8530003-…` RX | PC → device | write, write without response |
| `f8530004-…` DIAG | | read: Jacdac bus statistics + BLE counters |

Every Jacdac frame the device sends or receives is split into 20-byte chunks:
byte 0 = total number of chunks (`| 0x80` on the first chunk), byte 1 = chunks remaining after this
one, then up to 18 bytes of the frame. Frames from the PC use the same format. This matches
`createBluetoothTransport()` in jacdac-ts.

Sending never blocks the program: frames are queued (up to 10, further ones are dropped and
counted in DIAG) and sent while a client has notifications enabled.

## Pairing

The package doesn't change the Bluetooth security settings, so the project setting applies.
With the default (pairing required), put the device into pairing mode once (hold A+B, press and
release RESET) and pair it with the computer before connecting from the browser. For quick tests,
choose "No Pairing Required" in the project settings.

## Status

`jacdac.__bleStatus()` returns the transport state (the test program shows it on the LEDs):

| Value | Meaning |
|---|---|
| 4 | a client (e.g. the dashboard) is subscribed |
| 3 | waiting for a client |
| 2 | service registration failed, see the limit below |
| 1 | the Bluetooth stack isn't running |
| 0 | Jacdac hasn't been started |
| -1 | the transport isn't compiled in |

## Building locally

```sh
cd bluetooth-transport
npx makecode build --hw v3      # Calliope mini 3, hex in built/v3/binary.hex
```

`mkc.json` in this folder links `jacdac` to the parent folder, so the local sources are compiled.
Don't use `-c ../mkc-calliope.json` for this: current makecode CLI versions ignore its
`include: ["links.json"]`, and the `jacdac` dependency is then downloaded from GitHub instead.

## Limit: vendor UUID bases

The SoftDevice configuration allows only 4 vendor-specific 128-bit UUID bases
(`NRF_SDH_BLE_VS_UUID_COUNT`). DFU and partial flashing use 2 of them, Jacdac a third. Only one more
is left for either the micro:bit services of the bluetooth extension (accelerometer, LED, ...)
or the UART service; the HF2 console (`console.log` with the bluetooth extension) needs two.
Whatever registers after the fourth base silently doesn't work. If Jacdac is late,
`jacdac.__bleStatus()` returns 2. Start Jacdac (`jacdac.start()`) before other Bluetooth services
so it gets its base.

The count can be raised from a project's `pxt.json`: `"yotta": { "config": { "NRF_SDH_BLE_VS_UUID_COUNT": 6 } }`.
CODAL's `sdk_config.h` only sets it `#ifndef`, and the value reaches the build. Each base costs
SoftDevice RAM, though, and the RAM start is fixed by the linker script (`0x20002040`). Whether there is
enough room has not been tested on a device yet. If it isn't enough, the Bluetooth stack doesn't start
(`jacdac.__bleStatus()` returns 1).
