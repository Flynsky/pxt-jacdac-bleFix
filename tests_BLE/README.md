# Bluetooth test tools

Python scripts for testing a Calliope mini over Bluetooth LE from a Linux PC. They need
[bleak](https://github.com/hbldh/bleak) (`pip install bleak`) and BlueZ's `bluetoothctl`.
Both scripts find the Calliope by itself, pair it through BlueZ when it has to be paired (they tell you
when to put it into pairing mode: hold A+B, press RESET), and reconnect when the connection drops.

| Script | Tests |
|---|---|
| `calliope_uart.py` | the Bluetooth UART service (`bluetooth.startUartService()`) |
| `jacdac_ble.py` | Jacdac over Bluetooth (the `bluetooth-transport` extension, `JacdacBLE.cpp`) |

## jacdac_ble.py

`jacdac_ble.py` talks to the Jacdac GATT service (`f8530001-…`) without a browser. It reassembles the
20-byte BLE chunks into Jacdac frames, checks each frame's CRC, and decodes every packet: announces,
registers, events and commands. Service, register and event names come from the `*/constants.ts`
files in this repository. Once a second it prints a summary line: the devices it has seen (the
Calliope is marked `*`), frames/s, bytes/s, and CRC and chunk errors.

```bash
python3 jacdac_ble.py --selftest       # offline check of CRC, chunking and decoding (no device)
python3 jacdac_ble.py                  # print every frame and the summary
python3 jacdac_ble.py -q               # summary only
python3 jacdac_ble.py --identify       # PC -> Calliope: send "identify", the Calliope shows its animation
python3 jacdac_ble.py --raw --diag     # also hex-dump chunks/frames, read the diagnostics characteristic
python3 jacdac_ble.py -a CF:E7:78:91:66:AA --identify CV00   # a given Calliope / device (short id)
```

The Calliope has to run a program that uses the `jacdac` and `jacdac/bluetooth-transport`
extensions and starts Jacdac, e.g. `jacdac.start()` or any Jacdac module block. Whether pairing is
required depends on the project's Bluetooth setting.

If the tool can connect but finds no Jacdac service, it lists the services it did find and waits
for a program that has one. It does not pair or remove a stored pairing in that case. It only pairs
after it has asked you to use pairing mode (Calliope not found, or connections keep failing), or
when you pass `--pair`.

`--identify` with no argument targets the device that announces itself as a client, which is the
MakeCode program on the Calliope. Press Enter to send it again.
