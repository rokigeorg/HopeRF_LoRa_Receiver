//
// Created by Georg Rokita on 03.11.16.
//

#include <iostream>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "Labb_RFM95.h"

#define CHANNEL 0
#define RF95_FREQ 868100000 //868.1 MHz
#define RF95_SF 7

Labb_RFM95::Labb_RFM95(int cs_pin, int irq_pin, int RST_pin) {
    std::cout <<"Build instance of the Labb_RFM95.\n";

    wiringPiSetup();
    pinMode(cs_pin, OUTPUT);
    pinMode(irq_pin, INPUT);
    pinMode(RST_pin, OUTPUT);

    //int fd =
    wiringPiSPISetup(CHANNEL, 500000);

    _cs_pin =cs_pin;
    _irq_pin =irq_pin;
    _RST_pin= RST_pin;
    _mode = RHModeIdle;
    _freq = 868100000;      /////868.1 MHz
    _sf = 7;                ///SF 6 64 chips/symbol; SF 7 128 chips/symbol (default); SF 8 256 chips/symbol; SF 9 512 chips/symbol; SF 10 1024 chips/symbol; SF 11 2048 chips/symbol; SF 12 4096 chips/symbol

}

Labb_RFM95::~Labb_RFM95() {
}

bool Labb_RFM95::resetRFM95(){
    digitalWrite(_RST_pin, HIGH);
    delay(100);
    digitalWrite(_RST_pin, LOW);
    delay(100);
    return true;
}

void Labb_RFM95::selectreceiver()
{

    digitalWrite(_cs_pin, LOW);
}

void Labb_RFM95::unselectreceiver()
{
    digitalWrite(_cs_pin, HIGH);
}

uint8_t Labb_RFM95::readRegister(uint8_t addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void Labb_RFM95::writeRegister(uint8_t addr, uint8_t value)
{
    unsigned char spibuf[2];

    spibuf[0] = (addr | 0x80);
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}


void Labb_RFM95::printAllRegisters(){
    //registers I want to read
    uint8_t registers[] = {REG_FIFO, REG_OPMODE, RH_RF95_REG_02_RESERVED, RH_RF95_REG_03_RESERVED, RH_RF95_REG_04_RESERVED, RH_RF95_REG_05_RESERVED, RH_RF95_REG_06_FRF_MSB, RH_RF95_REG_07_FRF_MID, RH_RF95_REG_08_FRF_LSB , RH_RF95_REG_09_PA_CONFIG, RH_RF95_REG_0A_PA_RAMP, RH_RF95_REG_0B_OCP , RH_RF95_REG_0C_LNA ,
                           RH_RF95_REG_0D_FIFO_ADDR_PTR, RH_RF95_REG_0E_FIFO_TX_BASE_ADDR,RH_RF95_REG_0F_FIFO_RX_BASE_ADDR , RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, RH_RF95_REG_11_IRQ_FLAGS_MASK,RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_REG_13_RX_NB_BYTES, RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB,
                           RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB, RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB, RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB,RH_RF95_REG_18_MODEM_STAT,
                           RH_RF95_REG_19_PKT_SNR_VALUE,RH_RF95_REG_1A_PKT_RSSI_VALUE,RH_RF95_REG_1B_RSSI_VALUE,RH_RF95_REG_1C_HOP_CHANNEL,RH_RF95_REG_1D_MODEM_CONFIG1, RH_RF95_REG_1E_MODEM_CONFIG2, RH_RF95_REG_1F_SYMB_TIMEOUT_LSB, RH_RF95_REG_20_PREAMBLE_MSB ,RH_RF95_REG_21_PREAMBLE_LSB, RH_RF95_REG_22_PAYLOAD_LENGTH, RH_RF95_REG_23_MAX_PAYLOAD_LENGTH ,RH_RF95_REG_24_HOP_PERIOD, RH_RF95_REG_25_FIFO_RX_BYTE_ADDR,RH_RF95_REG_26_MODEM_CONFIG3};

    std::string registerNames[] = {"REG_FIFO","REG_OPMODE", "RH_RF95_REG_02_RESERVED", "RH_RF95_REG_03_RESERVED", "RH_RF95_REG_04_RESERVED", "RH_RF95_REG_05_RESERVED", "RH_RF95_REG_06_FRF_MSB", "RH_RF95_REG_07_FRF_MID", "RH_RF95_REG_08_FRF_LSB" , "RH_RF95_REG_09_PA_CONFIG", "RH_RF95_REG_0A_PA_RAMP", "RH_RF95_REG_0B_OCP", "RH_RF95_REG_0C_LNA" ,
                              "RH_RF95_REG_0D_FIFO_ADDR_PTR", "RH_RF95_REG_0E_FIFO_TX_BASE_ADDR","RH_RF95_REG_0F_FIFO_RX_BASE_ADDR" , "RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR", "RH_RF95_REG_11_IRQ_FLAGS_MASK","RH_RF95_REG_12_IRQ_FLAGS", "RH_RF95_REG_13_RX_NB_BYTES", "RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB",
                              "RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB", "RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB", "RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB","RH_RF95_REG_18_MODEM_STAT",
                              "RH_RF95_REG_19_PKT_SNR_VALUE","RH_RF95_REG_1A_PKT_RSSI_VALUE","RH_RF95_REG_1B_RSSI_VALUE","RH_RF95_REG_1C_HOP_CHANNEL","RH_RF95_REG_1D_MODEM_CONFIG1", "RH_RF95_REG_1E_MODEM_CONFIG2","RH_RF95_REG_1F_SYMB_TIMEOUT_LSB", "RH_RF95_REG_20_PREAMBLE_MSB","RH_RF95_REG_21_PREAMBLE_LSB", "RH_RF95_REG_22_PAYLOAD_LENGTH", "RH_RF95_REG_23_MAX_PAYLOAD_LENGTH" ,"RH_RF95_REG_24_HOP_PERIOD", "RH_RF95_REG_25_FIFO_RX_BYTE_ADDR","RH_RF95_REG_26_MODEM_CONFIG3"};


    unsigned int i;
    for( i=0; i < sizeof(registers);i++){

        std::cout << registerNames[i] << "Addr -> 0x";
        printf("%X", registers[i]);
        printf("%s",(char *)(": ") );
        printf("%X \n", readRegister(registers[i]) );

    }
}

void Labb_RFM95::setModeIdle() {
    if (_mode != RHModeIdle)
    {
        writeRegister(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
        _mode = RHModeIdle;
    }
}


void Labb_RFM95:: spiBurstRead(uint8_t * payload , uint8_t size)
{
    //uint8_t receivedCount = readRegister(RH_RF95_REG_13_RX_NB_BYTES);     //read register which tells the Number of received bytes
    uint8_t receivedbytes = size;

    for(int i = 0; i < receivedbytes; i++)
    {
        payload[i] = readRegister(RH_RF95_REG_00_FIFO);
        //_buf[i] = readRegister(REG_FIFO);
    }
}

void Labb_RFM95::handleInterrupt() {
    // Read the interrupt register
    uint8_t irq_flags = readRegister(RH_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
        _rxBad++;
    }
    else if (irq_flags & RH_RF95_RX_DONE) //_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE
    {
        // Have received a packet
        uint8_t len = readRegister(RH_RF95_REG_13_RX_NB_BYTES);

        // Reset the fifo read ptr to the beginning of the packet
        //writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
        //writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, (readRegister(RH_RF95_REG_25_FIFO_RX_BYTE_ADDR) - readRegister(RH_RF95_REG_13_RX_NB_BYTES)));

        uint8_t fiFo_Addr = readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);

        writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, fiFo_Addr);
        printf("FiFo Addr Ptr: %x\n", readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));

        spiBurstRead(_buf, len);
        _bufLen = len;
        writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

        // Remember the RSSI of this packet
        // this is according to the doc, but is it really correct?
        // weakest receiveable signals are reported RSSI at about -66
        _lastRssi = readRegister(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;
    }

}
