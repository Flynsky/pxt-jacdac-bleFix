namespace jacdac {
    /**
     * Diagnostic: state of the Jacdac Bluetooth transport (jacdac-bluetooth-transport).
     * -1 not compiled in, 0 Jacdac not started, 1 Bluetooth off,
     * 2 service registration failed (e.g. out of vendor UUID bases), 3 waiting for a client,
     * 4 client (e.g. the Jacdac dashboard) subscribed.
     **/
    //% shim=jacdac::__bleStatus
    export function __bleStatus(): number {
        return -1
    }
}
