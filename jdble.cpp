#include "pxt.h"
#include "jdlow.h"
#include "mbbridge.h"

// jacdac.__bleStatus() values
#define JDBLE_STATUS_NOT_COMPILED -1 // no jacdac-bluetooth-transport in the project
#define JDBLE_STATUS_NOT_STARTED 0   // Jacdac hasn't been started yet
#define JDBLE_STATUS_BLE_OFF 1       // the Bluetooth stack isn't running
#define JDBLE_STATUS_FAILED 2        // service registration failed (e.g. out of vendor UUID bases)
#define JDBLE_STATUS_READY 3         // waiting for a client to subscribe
#define JDBLE_STATUS_SUBSCRIBED 4    // a client (e.g. the Jacdac dashboard) receives frames

#if defined(MICROBIT_CODAL) && MICROBIT_CODAL
#include "MicroBitConfig.h"
#if CONFIG_ENABLED(DEVICE_BLE) && CONFIG_ENABLED(JACDAC_BLE_TRANSPORT)
#define JDBLE_COMPILED 1
#include "JacdacBLE.h"
#endif
#endif

namespace jacdac {

#ifdef JDBLE_COMPILED

static JacdacBLE *jdble;
static int jdbleStatus = JDBLE_STATUS_NOT_STARTED;
static void (*jdblePrevLog)(const uint8_t *data);

// Every frame the Jacdac stack sends or receives (pxt::logJDFrame, fiber context): pass it on to
// the previous logger (the WebUSB bridge), then queue it for the BLE client and start sending.
static void jdble_log(const uint8_t *data) {
    if (jdblePrevLog)
        jdblePrevLog(data);
    if (jdble->queueTx((const jd_frame_t *)data) == 0)
        jdble->pumpTx();
}

// more room in the SoftDevice's TX buffers, or the link changed
static void jdble_on_tx(MicroBitEvent) {
    jdble->pumpTx();
}

// frames from the PC: handled like frames from the WebUSB bridge (see sendExtFrame in app.cpp)
static void jdble_on_rx(MicroBitEvent) {
    LinkedFrame *f;
    while ((f = jdble->popRx()) != NULL) {
        pxt::sendJDFrame((const uint8_t *)&f->frame);
        free(f);
    }
}

void jdble_init() {
    if (jdble || jdbleStatus != JDBLE_STATUS_NOT_STARTED)
        return;

    if (!(uBit.bleManager.status & DEVICE_COMPONENT_RUNNING)) {
        jdbleStatus = JDBLE_STATUS_BLE_OFF;
        return;
    }

    jdble = new JacdacBLE();
    if (!jdble->ok()) {
        DMESG("JD_BLE: service registration failed");
        jdbleStatus = JDBLE_STATUS_FAILED;
        return;
    }

    // default listener flags: the handlers run in fiber context, not in the SoftDevice IRQ
    uBit.messageBus.listen(DEVICE_ID_JACDAC_BLE, JACDAC_BLE_EVT_RX, jdble_on_rx);
    uBit.messageBus.listen(DEVICE_ID_JACDAC_BLE, JACDAC_BLE_EVT_TX, jdble_on_tx);

    jdblePrevLog = pxt::logJDFrame;
    pxt::logJDFrame = jdble_log;
    jdbleStatus = JDBLE_STATUS_READY;
}

#endif // JDBLE_COMPILED

/**
 * Diagnostic: state of the Jacdac Bluetooth transport, see JDBLE_STATUS_*.
 */
//%
int __bleStatus() {
#ifdef JDBLE_COMPILED
    if (jdbleStatus == JDBLE_STATUS_READY && jdble->subscribed())
        return JDBLE_STATUS_SUBSCRIBED;
    return jdbleStatus;
#else
    return JDBLE_STATUS_NOT_COMPILED;
#endif
}

} // namespace jacdac
