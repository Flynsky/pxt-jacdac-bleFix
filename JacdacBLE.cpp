#include "pxt.h"
#include "jdlow.h"
#include "mbbridge.h"

#if defined(MICROBIT_CODAL) && MICROBIT_CODAL

#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE) && CONFIG_ENABLED(JACDAC_BLE_TRANSPORT)

#include "JacdacBLE.h"

using namespace jacdac;

// f8530001-a97f-49f5-a554-3e373fbea2d5, big-endian (RegisterBaseUUID reverses it)
const uint8_t JacdacBLE::base_uuid[16] = {0xf8, 0x53, 0x00, 0x01, 0xa9, 0x7f, 0x49, 0xf5,
                                          0xa5, 0x54, 0x3e, 0x37, 0x3f, 0xbe, 0xa2, 0xd5};

const uint16_t JacdacBLE::serviceUUID = 0x0001;
const uint16_t JacdacBLE::charUUID[mbbs_cIdxCOUNT] = {
    0x0002, // TX: device -> PC
    0x0003, // RX: PC -> device
    0x0004  // diagnostics
};

JacdacBLE::JacdacBLE()
    : rxLength(0), rxRemaining(0), rxActive(false), txCurrent(NULL), txSent(0), txTotal(0),
      txRemaining(0), txQueue(NULL), rxQueue(NULL) {
    memset(txChar, 0, sizeof(txChar));
    memset(rxChar, 0, sizeof(rxChar));
    memset(&diag, 0, sizeof(diag));

    // Errors here are silent in CODAL (e.g. when the SoftDevice is out of vendor UUID bases,
    // NRF_SDH_BLE_VS_UUID_COUNT); ok() checks the resulting handles.
    RegisterBaseUUID(base_uuid);
    CreateService(serviceUUID);

    CreateCharacteristic(mbbs_cIdxTX, charUUID[mbbs_cIdxTX], txChar, 1, JD_BLE_CHUNK_SIZE,
                         microbit_propNOTIFY);
    // the SoftDevice writes incoming chunks here; they are reassembled into rxFrame
    CreateCharacteristic(mbbs_cIdxRX, charUUID[mbbs_cIdxRX], rxChar, 1, JD_BLE_CHUNK_SIZE,
                         microbit_propWRITE | microbit_propWRITE_WITHOUT);
    // READAUTH: onDataRead() refreshes the bus statistics right before each read
    CreateCharacteristic(mbbs_cIdxDIAG, charUUID[mbbs_cIdxDIAG], (uint8_t *)&diag, sizeof(diag),
                         sizeof(diag), microbit_propREAD | microbit_propREADAUTH);
}

bool JacdacBLE::ok() {
    return bs_uuid_type != 0 && valueHandle(mbbs_cIdxTX) != 0 &&
           charHandles(mbbs_cIdxTX)->cccd != 0 && valueHandle(mbbs_cIdxRX) != 0;
}

bool JacdacBLE::subscribed() {
    return getConnectionHandle() != BLE_CONN_HANDLE_INVALID &&
           notifyChrValueEnabled(mbbs_cIdxTX);
}

int JacdacBLE::queueTx(const jd_frame_t *frame) {
    if (!subscribed())
        return -1; // nobody listening
    if (copyAndAppend(&txQueue, (jd_frame_t *)frame, JD_BLE_MAX_TX_QUEUE) < 0) {
        diag.tx_dropped++;
        return -1;
    }
    return 0;
}

void JacdacBLE::flushTx() {
    if (txCurrent) {
        free(txCurrent);
        txCurrent = NULL;
    }
    jd_crit_enter();
    LinkedFrame *q = txQueue;
    txQueue = NULL;
    jd_crit_exit();
    while (q) {
        LinkedFrame *next = q->next;
        free(q);
        q = next;
    }
}

void JacdacBLE::pumpTx() {
    for (;;) {
        if (!subscribed()) {
            flushTx();
            return;
        }

        if (!txCurrent) {
            jd_crit_enter();
            txCurrent = txQueue;
            if (txCurrent)
                txQueue = txCurrent->next;
            jd_crit_exit();
            if (!txCurrent)
                return;
            int size = JD_FRAME_SIZE(&txCurrent->frame);
            txTotal = (size + JD_BLE_DATA_SIZE - 1) / JD_BLE_DATA_SIZE;
            txRemaining = txTotal - 1;
            txSent = 0;
        }

        int size = JD_FRAME_SIZE(&txCurrent->frame);
        int n = size - txSent;
        if (n > JD_BLE_DATA_SIZE)
            n = JD_BLE_DATA_SIZE;

        uint8_t chunk[JD_BLE_CHUNK_SIZE];
        chunk[0] = (txTotal & 0x7f) | (txSent == 0 ? JD_BLE_FIRST_CHUNK_FLAG : 0);
        chunk[1] = txRemaining;
        memcpy(chunk + JD_BLE_HEADER_SIZE, (uint8_t *)&txCurrent->frame + txSent, n);
        uint16_t len = n + JD_BLE_HEADER_SIZE;

        ble_gatts_hvx_params_t hvx;
        memset(&hvx, 0, sizeof(hvx));
        hvx.handle = valueHandle(mbbs_cIdxTX);
        hvx.type = BLE_GATT_HVX_NOTIFICATION;
        hvx.p_len = &len;
        hvx.p_data = chunk;

        uint32_t err = sd_ble_gatts_hvx(getConnectionHandle(), &hvx);
        if (err == NRF_SUCCESS) {
            txSent += n;
            if (txRemaining)
                txRemaining--;
            if (txSent >= size) {
                free(txCurrent);
                txCurrent = NULL;
                diag.tx_frames++;
            }
        } else if (err == NRF_ERROR_RESOURCES || err == NRF_ERROR_BUSY) {
            // SoftDevice TX buffers are full; BLE_GATTS_EVT_HVN_TX_COMPLETE resumes us
            return;
        } else {
            // disconnected, notifications turned off, ...: drop everything, never retry forever
            flushTx();
            return;
        }
    }
}

LinkedFrame *JacdacBLE::popRx() {
    jd_crit_enter();
    LinkedFrame *f = rxQueue;
    if (f)
        rxQueue = f->next;
    jd_crit_exit();
    return f;
}

void JacdacBLE::rxReset() {
    rxActive = false;
    rxLength = 0;
    rxRemaining = 0;
}

// SoftDevice event IRQ: reassemble, queue, and let a fiber deliver the frame (JACDAC_BLE_EVT_RX)
void JacdacBLE::rxChunk(const uint8_t *data, uint16_t len) {
    if (len < JD_BLE_HEADER_SIZE || len > JD_BLE_CHUNK_SIZE) {
        diag.rx_errors++;
        rxReset();
        return;
    }

    if (data[0] & JD_BLE_FIRST_CHUNK_FLAG) {
        if (rxActive)
            diag.rx_errors++; // previous frame never completed
        rxActive = true;
        rxLength = 0;
        rxRemaining = data[0] & 0x7f;
    }

    if (!rxActive || rxRemaining == 0) {
        // chunk without a first chunk (e.g. after an error): wait for the next frame
        diag.rx_errors++;
        rxReset();
        return;
    }

    rxRemaining--;
    uint16_t n = len - JD_BLE_HEADER_SIZE;
    if (data[1] != rxRemaining || rxLength + n > sizeof(rxFrame)) {
        diag.rx_errors++;
        rxReset();
        return;
    }

    memcpy((uint8_t *)&rxFrame + rxLength, data + JD_BLE_HEADER_SIZE, n);
    rxLength += n;
    if (rxRemaining > 0)
        return;

    rxActive = false;
    unsigned frameSize = JD_FRAME_SIZE(&rxFrame);
    if (rxLength < JD_SERIAL_FULL_HEADER_SIZE || frameSize > rxLength ||
        frameSize > sizeof(jd_frame_t)) {
        diag.rx_errors++;
        return;
    }
    if (copyAndAppend(&rxQueue, &rxFrame, JD_BLE_MAX_RX_QUEUE) < 0) {
        diag.rx_errors++;
        return;
    }
    diag.rx_frames++;
    MicroBitEvent(DEVICE_ID_JACDAC_BLE, JACDAC_BLE_EVT_RX);
}

void JacdacBLE::onDataWritten(const microbit_ble_evt_write_t *params) {
    if (params->handle == valueHandle(mbbs_cIdxRX))
        rxChunk(params->data, params->len);
}

void JacdacBLE::onDataRead(microbit_onDataRead_t *params) {
    if (params->handle != valueHandle(mbbs_cIdxDIAG))
        return;
    // the value lives in `diag` (is_value_user); refresh it at the start of each (long) read
    if (params->offset == 0)
        memcpy(&diag.bus, jd_get_diagnostics(), sizeof(diag.bus));
    params->update = false;
}

void JacdacBLE::onDisconnect(const microbit_ble_evt_t *) {
    rxReset();
    // the pump sees there is no subscriber any more and frees the queue
    if (txPending())
        MicroBitEvent(DEVICE_ID_JACDAC_BLE, JACDAC_BLE_EVT_TX);
}

bool JacdacBLE::onBleEvent(const microbit_ble_evt_t *p_ble_evt) {
    if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_HVN_TX_COMPLETE && txPending())
        MicroBitEvent(DEVICE_ID_JACDAC_BLE, JACDAC_BLE_EVT_TX);
    return MicroBitBLEService::onBleEvent(p_ble_evt);
}

#endif // DEVICE_BLE && JACDAC_BLE_TRANSPORT
#endif // MICROBIT_CODAL
