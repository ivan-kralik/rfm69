#ifndef RFM69_REGISTERS_H
#define RFM69_REGISTERS_H

#define RFM69_REG_FIFO 0x00
#define RFM69_REG_MODE 0x01
#define RFM69_REG_FRF_MSB 0x07
#define RFM69_REG_FRF_MID 0x08
#define RFM69_REG_FRF_LSB 0x09
#define RFM69_REG_OSC_1 0x0A
#define RFM69_REG_LISTEN_1 0x0D
#define RFM69_REG_LISTEN_2 0x0E
#define RFM69_REG_LISTEN_3 0x0F
#define RFM69_REG_PA_LEVEL 0x11
#define RFM69_REG_OCP 0x13
#define RFM69_REG_RSSI_CONFIG 0x23
#define RFM69_REG_RSSI_VALUE 0x24
#define RFM69_REG_DIO_MAPPING_1 0x25
#define RFM69_REG_IRQ_FLAGS_1 0x27
#define RFM69_REG_IRQ_FLAGS_2 0x28
#define RFM69_REG_RSSI_THRESH 0x29
#define RFM69_REG_RX_TIMEOUT_2 0x2B
#define RFM69_REG_SYNC_CONFIG 0x2E
#define RFM69_REG_SYNC_VALUE_1 0x2F
#define RFM69_REG_PACKET_CONFIG_1 0x37
#define RFM69_REG_AUTO_MODES 0x3B
#define RFM69_REG_FIFO_THRESH 0x3C
#define RFM69_REG_PACKET_CONFIG_2 0x3D
#define RFM69_REG_PAYLOAD_LENGTH 0x38
#define RFM69_REG_NODE_ADRS 0x39
#define RFM69_REG_BROADCAST_ADRS 0x3A
#define RFM69_REG_TEST_PA_1 0x5A
#define RFM69_REG_TEST_PA_2 0x5C
#define RFM69_REG_TEST_DAGC 0x6F

#define RFM69_FXOSC 32000000 // Hz
#define RFM69_FSTEP 61 // Hz (FXOSC / 2^19)

#define RFM69_MODE_SLEEP 0x00
#define RFM69_MODE_STBY 0x04
#define RFM69_MODE_FS 0x08
#define RFM69_MODE_TX 0x2C
#define RFM69_MODE_RX 0x10
#define RFM69_MODE_SLEEP_LISTEN 0x40
#define RFM69_MODE_STBY_LISTEN 0x44
#define RFM69_MODE_RX_LISTEN 0x50

#endif
