#ifndef JACDAC_BLE_SERVICE_H
#define JACDAC_BLE_SERVICE_H

#if defined(MICROBIT_CODAL) && MICROBIT_CODAL

#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE) && CONFIG_ENABLED(JACDAC_BLE_TRANSPORT)

#include "MicroBitBLEManager.h"
#include "MicroBitBLEService.h"
#include "jdlow.h"
#include "mbbridge.h"

#define DEVICE_ID_JACDAC_BLE 3056
// a complete frame was received from the PC and is waiting in the RX queue
#define JACDAC_BLE_EVT_RX 1
// the SoftDevice has room for more notifications (or the link state changed): pump the TX queue
#define JACDAC_BLE_EVT_TX 2

// ATT MTU is 23 on CODAL (NRF_SDH_BLE_GATT_MAX_MTU_SIZE), so a notification/write carries 20 bytes.
// Each chunk: [0] total chunk count (7 bits) | 0x80 on the first chunk, [1] chunks remaining after
// this one, then up to 18 bytes of the Jacdac frame. Same framing as jacdac-ts transport/bluetooth.ts.
#define JD_BLE_CHUNK_SIZE 20
#define JD_BLE_HEADER_SIZE 2
#define JD_BLE_DATA_SIZE (JD_BLE_CHUNK_SIZE - JD_BLE_HEADER_SIZE)
#define JD_BLE_FIRST_CHUNK_FLAG 0x80

// frames waiting to go out over BLE; when full, new frames are dropped (and counted)
#define JD_BLE_MAX_TX_QUEUE 10
// complete frames from the PC waiting to be handed to the Jacdac stack
#define JD_BLE_MAX_RX_QUEUE 5

// what the diagnostics characteristic (0x0004) returns
struct JacdacBLEDiagnostics {
    jd_diagnostics_t bus;   // Jacdac wire statistics, see jdlow.h
    uint32_t tx_frames;     // frames sent to the PC
    uint32_t tx_dropped;    // frames dropped because the TX queue was full
    uint32_t rx_frames;     // frames received from the PC
    uint32_t rx_errors;     // malformed or out-of-order chunks / frames from the PC
};

/**
 * GATT service carrying Jacdac frames between the device and a PC (e.g. the Jacdac dashboard over
 * Web Bluetooth). Service f8530001-a97f-49f5-a554-3e373fbea2d5:
 *   0x0002 TX   device -> PC   NOTIFY
 *   0x0003 RX   PC -> device   WRITE, WRITE WITHOUT RESPONSE
 *   0x0004 DIAG               READ (JacdacBLEDiagnostics)
 *
 * Never blocks: frames are queued and sent by pumpTx() from fiber context; received frames are
 * reassembled in the SoftDevice event IRQ, queued, and delivered by popRx() from fiber context.
 */
class JacdacBLE : public MicroBitBLEService {
  public:
    JacdacBLE();

    // service and characteristics were registered with the SoftDevice
    bool ok();
    // a client is connected and has enabled notifications on TX
    bool subscribed();

    // Queue a frame for the PC. Safe from fibers and IRQs. Returns 0, or -1 if dropped.
    int queueTx(const jd_frame_t *frame);
    // Send as many queued chunks as the SoftDevice accepts. Fiber context only.
    void pumpTx();
    // Next frame received from the PC (free() it when done), or NULL. Fiber context only.
    jacdac::LinkedFrame *popRx();

    // Index for each characteristic in arrays of handles and UUIDs
    typedef enum mbbs_cIdx { mbbs_cIdxTX, mbbs_cIdxRX, mbbs_cIdxDIAG, mbbs_cIdxCOUNT } mbbs_cIdx;

    // UUIDs for our service and characteristics
    static const uint8_t base_uuid[16];
    static const uint16_t serviceUUID;
    static const uint16_t charUUID[mbbs_cIdxCOUNT];

    // Data for each characteristic when they are held by Soft Device.
    MicroBitBLEChar chars[mbbs_cIdxCOUNT];

    int characteristicCount() { return mbbs_cIdxCOUNT; };
    MicroBitBLEChar *characteristicPtr(int idx) { return &chars[idx]; };

    bool onBleEvent(const microbit_ble_evt_t *p_ble_evt);
    void onDisconnect(const microbit_ble_evt_t *p_ble_evt);

  protected:
    void onDataWritten(const microbit_ble_evt_write_t *params);
    void onDataRead(microbit_onDataRead_t *params);

  private:
    // characteristic values (owned by us, is_value_user)
    uint8_t txChar[JD_BLE_CHUNK_SIZE];
    uint8_t rxChar[JD_BLE_CHUNK_SIZE];
    JacdacBLEDiagnostics diag;

    // RX reassembly (SoftDevice event IRQ only)
    jd_frame_t rxFrame;
    uint16_t rxLength;
    uint8_t rxRemaining; // chunks still expected for the current frame
    bool rxActive;       // inside a frame; false = waiting for a first chunk

    // frame currently being sent (fiber context only)
    jacdac::LinkedFrame *txCurrent;
    uint16_t txSent;
    uint8_t txTotal;
    uint8_t txRemaining;

    jacdac::LinkedFrame *volatile txQueue;
    jacdac::LinkedFrame *volatile rxQueue;

    void rxReset();
    void rxChunk(const uint8_t *data, uint16_t len);
    void flushTx();
    bool txPending() { return txCurrent != NULL || txQueue != NULL; }
};

#endif // DEVICE_BLE && JACDAC_BLE_TRANSPORT
#endif // MICROBIT_CODAL
#endif
