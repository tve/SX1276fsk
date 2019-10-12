// RFM96 / sx1276 FSK driver
// Copyright (c) 2019 by Thorsten von Eicken, see LICENSE file

#include <sys/time.h>

class SX1276fsk {
public:
    SX1276fsk(SPIClass &spi_, int8_t ss_, int8_t reset_=-1)
        : spi(spi_)
        , ss(ss_)
        , reset(reset_)
    {};

    void init(uint8_t id, uint8_t group, int freq);
    void setIntrPins(int8_t dio0, int8_t dio4);
    void txPower(uint8_t level);

    // receive initializes the radio for RX if it's not in RX mode, else it checks whether a
    // packet is in the FIFO and pulls it out if it is. The returned value is -1 if there is no
    // packet, else the length of the packet, which includes the destination address, source
    // address, and payload bytes, but excludes the length byte itself. The fei, rssi, and lna
    // values are also valid when a packet has been received but may change with the next call
    // to receive().
    int receive(void* ptr, int len);

    // send transmits the packet as specified by the header, which consists of the destination address
    // in the lower 6 bits and bit 7 for ?? as well as bit 6 for ??.
    // Note: the code is limited to len <62 because the FIFO is filled before initiating TX.
    bool send(uint8_t header, const void* ptr, int len);

    // sleep puts the radio to sleep to save power
    void sleep();

    struct timeval rxAt; // timestamp of packet reception
    int32_t afc;    // AFC freq correction applied
    uint8_t snr;    // in dB
    uint8_t rssi;   // -RSSI*2 of last packet received
    uint8_t lna;    // LNA attenuation in dB
    uint8_t myId;
    uint8_t parity;

//private: // not enforced...

    uint8_t readReg(uint8_t addr) { return rwReg(addr, 0); }
    void writeReg(uint8_t addr, uint8_t val) { rwReg(addr | 0x80, val); }
    uint8_t rwReg(uint8_t cmd, uint8_t val);

    enum {
        REG_FIFO          = 0x00,
        REG_OPMODE        = 0x01,
        REG_FRFMSB        = 0x06,
        REG_PACONFIG      = 0x09,
        REG_LNAVALUE      = 0x0C,
        REG_RXCONFIG      = 0x0D,
        REG_RSSITHRES     = 0x10,
        REG_AFCFEI        = 0x1A,
        REG_AFCMSB        = 0x1B,
        REG_AFCLSB        = 0x1C,
        REG_FEIMSB        = 0x1D,
        REG_FEILSB        = 0x1E,
        REG_RSSIVALUE     = 0x11,
        REG_IRQFLAGS1     = 0x3E,
        REG_IRQFLAGS2     = 0x3F,
        REG_DIOMAPPING1   = 0x40,
        REG_DIOMAPPING2   = 0x41,
        REG_SYNCVALUE1    = 0x28,
        REG_SYNCVALUE3    = 0x2A,
        REG_NODEADDR      = 0x33,
        REG_BCASTADDR     = 0x34,
        REG_PADAC         = 0x4D,
        //REG_FIFOTHRESH    = 0x3C,
        //REG_PKTCONFIG2    = 0x3D,

        MODE_SLEEP        = 0,
        MODE_STANDBY      = 1,
        MODE_FSTX         = 2,
        MODE_TRANSMIT     = 3,
        MODE_FSRX         = 4,
        MODE_RECEIVE      = 5,

        //START_TX          = 0xC2,
        //STOP_TX           = 0x42,

        //RCCALSTART        = 0x80,
        IRQ1_MODEREADY    = 1<<7,
        IRQ1_RXREADY      = 1<<6,
        IRQ1_PREAMBLEDETECT=1<<1,
        IRQ1_SYNADDRMATCH = 1<<0,

        IRQ2_FIFONOTEMPTY = 1<<6,
        IRQ2_PACKETSENT   = 1<<3,
        IRQ2_PAYLOADREADY = 1<<2,
    };

    void setMode (uint8_t newMode);
    bool modeReady();
    void configure (const uint8_t* p);
    void setFrequency (uint32_t freq);
    void readRSSI();
    int readPacket(void* ptr, int len);
    void restartRx();
    bool transmitting();
    bool receiving();
    //void interrupt0();
    //void interrupt4();

    uint8_t mode; // last operation mode programmed into the radio
    SPIClass &spi;
    int8_t ss, reset, dio0, dio4; // pin numbers
    uint8_t lastFlag; //
    uint32_t rssiAt;  // timestamp when RSSI was captured
    uint32_t  bgRssiAt; // last bgRssi measurement (micros())
    uint16_t  bgRssi; // background (noise) RSSI

};


int encodeVarints(int32_t ints[], int count, uint8_t *buf, int len);
int decodeVarints(uint8_t buf[], int len, int32_t *ints, int count);
int decodeVarint(uint8_t buf[], int len, int32_t *value);
