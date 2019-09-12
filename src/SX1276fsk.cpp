// RFM96 / sx1276 FSK driver
// Copyright (c) 2019 by Thorsten von Eicken, see LICENSE file

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <SPI.h>
#include "SX1276fsk.h"

static SPISettings spiSettings;

uint8_t SX1276fsk::rwReg(uint8_t cmd, uint8_t val) {
    spi.beginTransaction(spiSettings);
    digitalWrite(ss, LOW);
    spi.write(cmd);
    uint8_t r = spi.transfer(val);
    digitalWrite(ss, HIGH);
    spi.endTransaction();
    return r;
}

void SX1276fsk::setMode (uint8_t newMode) {
    // disable interrupts (dio0->none)
    writeReg(REG_DIOMAPPING1, 0xB0);
    mode = newMode;
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0x7) | newMode);
    if (mode == MODE_RECEIVE) {
        writeReg(REG_IRQFLAGS1, 0xff);
        // enable interrupts (dio0->payload-ready, dio4->preamble)
        writeReg(REG_DIOMAPPING1, 0x0C);
        // ensure RX really restarts and does AFC,AGC, etc.
        writeReg(REG_RXCONFIG, 0x9E|0x40); // restart RX
        writeReg(REG_AFCFEI, 3); // clear AFC regs
    }
}

// FS->TX takes 65us, FS->RX takes 100us
bool SX1276fsk::modeReady() { return (readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) != 0; }

void SX1276fsk::setFrequency (uint32_t hz) {
    // accept any frequency scale as input, including KHz and MHz
    // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
    while (hz < 100000000)
        hz *= 10;

    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
    // due to this, the lower 6 bits of the calculated factor will always be 0
    // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
    // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
    uint32_t frf = (hz << 2) / (32000000L >> 11);
    writeReg(REG_FRFMSB, frf >> 10);
    writeReg(REG_FRFMSB+1, frf >> 2);
    writeReg(REG_FRFMSB+2, frf << 6);
}

void SX1276fsk::configure (const uint8_t* p) {
    while (true) {
        uint8_t cmd = p[0];
        if (cmd == 0)
            break;
        writeReg(cmd, p[1]);
        p += 2;
    }
    mode = MODE_SLEEP;
}

// configRegs contains register-address, register-value pairs for initialization.
// Note that these are TvE's values, not JCW's, specifically, RxBW and Fdev have been optimized
// for rf69/rf96 and may not work with rf12's. Also, the sync bytes include one preamble byte to
// reduce the number of false sync word matches due to noise. These settings are compatible with the
// Go driver in https://github.com/tve/devices/tree/master/sx1231
static const uint8_t RF96configRegs [] = {
    0x01, 0x00, // FSK mode, high-freq regs, sleep mode
    0x01, 0x00, // FSK mode, high-freq regs, sleep mode
    0x02, 0x02, 0x03, 0x8A, // Bit rate: 49230bps
    0x04, 0x03, 0x05, 0x4E, // 51.5kHzFdev -> modulation index = 2.1
    0x09, 0xF0+11, // use PA_BOOST, start at 13dBm
    0x0A, 0x09, // no shaping, 40us TX rise/fall
    0x0B, 0x32, // Over-current protection @150mA
    0x0C, 0x20, // max LNA gain, no boost
    0x0D, 0x9E, // AFC on, AGC on, AGC&AFC on preamble detect
    0x0E, 0x04, // 32-sample rssi smoothing (5 bit times)
    0x0F, 0x0A, // 10dB RSSI collision threshold
    0x10, 90*2, // RSSI threshold
    0x12, 0x52, // RxBW 83kHz
    0x13, 0x4A, // AfcBw 125kHz
    0x1A, 0x01, // clear AFC at start of RX
    0x1F, 0xCA, // 3 byte preamble detector, tolerate 10 chip errors (2.5 bits)
    0x20, 0x00, // No RX timeout if RSSI doesn't happen
    0x21, 0x00, // No RX timeout if no preamble
    0x22, 0x00, // No RX timeout if no sync
    0x23, 0x02, // delay 8 bits after RX end before restarting RX
    0x24, 0x07, // no clock out
    0x25, 0x00, 0x26, 0x05, // preamble 5 bytes
    0x27, 0x12, // no auto-restart, 0xAA preamble, enable 3 byte sync
    0x28, 0xAA, // sync1: same as preamble, gets us additional check
    0x29, 0x2D, 0x2A, 0x2A, // sync2 (fixed), and sync3 (network group)
    0x30, 0xD0, // whitening, CRC on, no addr filt, CCITT CRC
    0x31, 0x40, // packet mode
    0x32, 255,  // max RX packet length
    0x35, 0x8F, // start TX when FIFO has 1 byte, FifoLevel intr when 15 bytes in FIFO
    0x41, 0xF1, // dio5->mode-ready, dio4->preamble-detect intr
    0x44, 0x00, // no fast-hop
    0x4D, 0x07, // enable 20dBm tx power
    0
};

void SX1276fsk::init (uint8_t id, uint8_t group, int freq) {
    myId = id;
    dio0 = -1;
    dio4 = -1;

    // b7 = group b7^b5^b3^b1, b6 = group b6^b4^b2^b0
    parity = group ^ (group << 4);
    parity = (parity ^ (parity << 2)) & 0xC0;

    if (reset > 0) {
        digitalWrite(reset, LOW);
        delay(10);
        digitalWrite(reset, HIGH);
        delay(20);
    }

    do
        writeReg(REG_SYNCVALUE1, 0xAA);
    while (readReg(REG_SYNCVALUE1) != 0xAA);
    do
        writeReg(REG_SYNCVALUE1, 0x55);
    while (readReg(REG_SYNCVALUE1) != 0x55);

    configure(RF96configRegs);
    setFrequency(freq);

    writeReg(REG_SYNCVALUE3, group);

}

void IRAM_ATTR SX1276fsk::interrupt0() { intr0 = true; }
void IRAM_ATTR SX1276fsk::interrupt4() { intr4 = true; }

void SX1276fsk::setIntrPins(int8_t dio0_, int8_t dio4_) {
    if (dio0_ != dio0) {
        if (dio0 >= 0) detachInterrupt(dio0);
        dio0 = dio0_;
        intr0 = false;
        if (dio0 >= 0) {
            attachInterrupt(dio0, std::bind(&SX1276fsk::interrupt0, this), RISING);
        }
    }
    if (dio4_ != dio4) {
        if (dio4 >= 2) detachInterrupt(dio4);
        dio4 = dio4_;
        intr4 = false;
        if (dio4 >= 0) attachInterrupt(dio4, std::bind(&SX1276fsk::interrupt4, this), RISING);
    }
}

// txPower sets the transmit power to the requested level in dB. The driver assumes that the
// PA_BOOST pin is used in the radio module and allows adjustment from 2dBm to 20dBm.
void SX1276fsk::txPower (uint8_t level) {
    if (level < 2) level = 2;
    if (level > 20) level = 20;
    setMode(MODE_STANDBY);
    if (level > 17) {
        writeReg(REG_PADAC, 0x87); // turn 20dBm mode on
        writeReg(REG_PACONFIG, 0xf0+level-5);
    } else {
        writeReg(REG_PACONFIG, 0xf0+level-2);
        writeReg(REG_PADAC, 0x84); // turn 20dBm mode off
    }
}

void SX1276fsk::sleep () {
    setMode(MODE_SLEEP);
}

static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };

void SX1276fsk::readRSSI() {
    rssi = readReg(REG_RSSIVALUE);
    lna = RF96lnaMap[ (readReg(REG_LNAVALUE) >> 5) & 0x7 ];
    int16_t f = (uint16_t)readReg(REG_AFCMSB);
    f = (f<<8) | (uint16_t)readReg(REG_AFCLSB);
    afc = (int32_t)f * 61;
    rssiAt = millis();
}

int SX1276fsk::readPacket(void* ptr, int len) {
    spi.beginTransaction(spiSettings);
    digitalWrite(ss, LOW);
    spi.write(REG_FIFO);
    uint8_t count = spi.transfer(0); // first byte of packet is length
    if (count <= len) {
        spi.transferBytes((uint8_t*)ptr, (uint8_t*)ptr, count);
    } else {
        spi.transferBytes((uint8_t*)ptr, (uint8_t*)ptr, len);
        for (int i=len; i<count; i++) spi.transfer(0);
    }
    digitalWrite(ss, HIGH);
    spi.endTransaction();
    // flag stale RSSI
    if (millis()-rssiAt > 60) {
        printf("!RSSI stale:%ld!", millis()-rssiAt);
        rssi = 0;
        afc = 0;
        lna = 0;
    }
    // need to restartRX to get proper fresh AFC/AGC
    restartRx();

    // only accept packets intended for us, or broadcasts
    // ... or any packet if we're the special catch-all node
    uint8_t dest = *(uint8_t*) ptr;
    if ((dest & 0xC0) == parity) {
        uint8_t destId = dest & 0x3F;
        if (destId == myId || destId == 0 || myId == 63)
            return count;
    }
    return -1;
}

void SX1276fsk::restartRx() {
    intr0 = false;
    intr4 = false;
    lastFlag = 0;
    rssiAt = 0;
    writeReg(REG_RXCONFIG, 0x9E|0x40); // restart RX
}

bool SX1276fsk::transmitting() {
    if (mode != MODE_TRANSMIT) return false;
    if ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0) return true;
    mode = MODE_STANDBY; // TODO: verify that radio auto-switches to standby
    return false;
}

bool SX1276fsk::receiving() {
    if (mode != MODE_RECEIVE) return false;
    if ((readReg(REG_IRQFLAGS1) & IRQ1_SYNADDRMATCH) != 0) return true;
    return false;
}

int SX1276fsk::receive(void* ptr, int len) {
    if (transmitting()) return false;

    if (mode != MODE_RECEIVE) {
        restartRx();
        rssi = 0;
        lna = 0;
        afc = 0;
        setMode(MODE_RECEIVE);
        return -1;
    }

    // if a packet has started, read the RSSI, AFC, etc stats
    if (intr4) {
        // got an SyncAddr interrupt, need to read RSSI, AFC, etc.
        rssiAt = millis();
        readRSSI();
        //printf("!RSSI%d!", -rssi/2);
        intr4 = false;
    } else if (dio4 < 0) {
        // no interrupt pin for preamble detect, need to check explicitly
        if ((readReg(REG_IRQFLAGS1) & IRQ1_PREAMBLEDETECT) != lastFlag) {
            lastFlag ^= IRQ1_PREAMBLEDETECT;
            if (lastFlag) {
                readRSSI();
                //printf("[RSSI%d]", -rssi/2);
            }
        }
    }

    // if a packet has been received, fetch it
    if (intr0) {
        // got a payloadReady interrupt, need to read packet
        intr0 = false;
        return readPacket(ptr, len);
    } else if (dio0 < 0) {
        // no interrupt pin for payloadReady, need to check explicitly
        if (readReg(REG_IRQFLAGS2) & IRQ2_PAYLOADREADY) {
            //printf("[RX]");
            return readPacket(ptr, len);
        }
    }

    // if we don't get a packet (sync address match) within a few ms restart RX so we end up getting
    // a fresh AFC/AGC for the next actual packet.
    // Preamble+sync is 5+3=8 bytes, @49230 baud that's 1.3ms.
    if (rssiAt != 0 && millis()-rssiAt > 4 && (readReg(REG_IRQFLAGS1)&IRQ1_SYNADDRMATCH) == 0) {
        restartRx();
    }

    return -1;
}

bool SX1276fsk::send (uint8_t header, const void* ptr, int len) {
    if (receiving()) return false;

    setMode(MODE_FSTX);

    spi.beginTransaction(spiSettings);
    digitalWrite(ss, LOW);
    spi.write(REG_FIFO|0x80);
    spi.transfer(len + 2);
    spi.transfer((header & 0x3F) | parity);
    spi.transfer((header & 0xC0) | myId);
    spi.writeBytes((uint8_t*)ptr, len);
    digitalWrite(ss, HIGH);
    spi.endTransaction();

    setMode(MODE_TRANSMIT);
    return true;
}
