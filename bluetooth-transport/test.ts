// Test program for the Jacdac Bluetooth transport.
// The LED matrix shows the transport state (see README.md):
// 4 = client (dashboard) subscribed, 3 = waiting for a client, 2 = service registration failed,
// 1 = Bluetooth off, 0 = Jacdac not started, -1 = transport not compiled in.
jacdac.start()

basic.forever(() => {
    basic.showNumber(jacdac.__bleStatus())
    basic.pause(500)
})
