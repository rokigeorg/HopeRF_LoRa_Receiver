//
// Created by Georg Rokita on 03.11.16.
// This Class includes code from Thomas Telkamp thomas@telkamp.eu and the Single Channel LoRaWAN Gateway and also the radio Library RFM95
// It implies the following dependencies
// SPI  enabled on the Raspberry Pi
// WiringPi: a GPIO access library written in C for the BCM2835 used in the Raspberry Pi. sudo apt-get install wiringpi see http://wiringpi.com
//

#ifndef HOPERF_LORA_RECEIVER_LABB_RFM95_H
#define HOPERF_LORA_RECEIVER_LABB_RFM95_H


#include <cstdint>
#include "RFM95registers.h"

#define CHANNEL 0

#define RF95_SYMB_TIMEOUT   0x64 //0x08
#define RF95_MAX_PAYLOAD_LENGTH 0xff    //default accepted payload length -> all packages can be received
#define PAYLOAD_LENGTH 0x40
#define FREQ_HOP_PERIOD 0x00 //0x00 means freq hopping is turned off
//define LNA_MAX_GAIN //LowNoiseAmplifier Gain if the gateway is not set in automatic gain controll

#define SX1276_MODE_Continuos 0x85

#define CR_4_5 5
#define CR_4_6 6
#define CR_4_7 7
#define CR_4_8 8



class Labb_RFM95 {

public:


    /**Creates an instance of the Labb_RFM95
     * cs_pin -> Chipselect pin on the RPI Model B
     * irq_pin -> Interrupt pin on the RPI, On the Adafruit RFM9x is this pin labeled dio0
     * RST_pin -> Reset pin on the RPI
    */
    Labb_RFM95(int cs_pin = 6, int irq_pin = 7, int RST_pin = 0);

    /**
     * Destructor
     */

    ~Labb_RFM95();

    /**
    * This Function prints out all the registers of the RFM95
    */
    void printAllRegisters();

    /**
     * This function reads the register from the RFM95.
     * @param addr -> takes an address of a register
     * @return -> returns the current settings / byte / value of the register address
     */
    uint8_t readRegister(uint8_t addr);

    /**
     * This function writes over SPI a register
     * @param addr -> register address
     * @param value -> value which the selected register should be set to
     */
    void writeRegister(uint8_t addr, uint8_t value);

    /**
     * This function sets the connected SPI Chipselcet pin to LOW to selctet the chip for SPI communication
     */
    void selectreceiver();

    /**
    * This function sets the connected SPI Chipselcet pin to HIGH to unselctet the chip for SPI communication.
    */
    void unselectreceiver();

    /**
     * This funcition sets the RF95 into Idle Mode
     */
    void setModeIdle();

    /**
     * This function reads the data stream from the SPI interface and stores it in the given byte Array (Payload) until end of the given array is near
     * @param payload -> array where the data from the SPI Stream should be written to
     * @param size -> size of the passed payload array
     */
    void spiBurstRead(uint8_t *payload, uint8_t size);

    /**
     * This function resets the RFM95 Modul via toggling the RST pin
     * @return -> TRUE when it is done
     */
    bool resetRFM95();

    /**
     * This function sets the RF95 to the given frequency
     * @param freq -> frequency in this formate please 868100000 for 868.1 MHz
     * @return -> returns true when its done
     */
    bool setFrequency(uint32_t freq);

    /**
     * This function sets up the Modem registers of the RFM95
     * For details please look in the data sheet of the RFM95 in the LoRa Register Table for the following registers
     * REG 1D MODEM_CONFIG1
     * REG 1E MODEM_CONFIG2
     * REG 26 MODEM_CONFIG3 -> [7-4 bit: unused][3 bit: 0->static node / 1->mobile node] [2 bit: 0->LNA gain set by register LnaGain / 1->LNA gain set by the internal AGC loop][1-0 bit: reserved]
     */
    void setModemRegisters();


    /**
     * This functions is suppose to handle an Interrup. The RFM95 toggles an interrupt pin (dio0) when a LoRa package is received.
     * This function therefore calls all functions which are nessecary to read the FiFo buffer of the RF95.
     * In my opinion its pretty imporant understand it.
     */
    void handleInterrupt();


    /**
     * This function sets the RX operation time-out value expressed as number of symbols: TimeOut = SymbTimeout·Ts
     * @param timeOutPeriod
     */
    void setSymbTimeout(uint8_t timeOutPeriod);


    /**
     * This function sets the maximum payload length; Used in Explicit Header Mode.
     * if header payload length exceeds value a header CRC error is generated. Allows filtering of packet with a bad size.
     * Here the header provides information on the payload, namely:   The payload length in bytes. The forward error correction code rateThe presence of an optional 16-bits CRC for the payload.
     * The header is transmitted with maximum error correction code (4/8). It also has its own CRC to allow the receiver to discard invalid headers.
     * @param mPayloadLength
     */
    void setMaxPayloadLength(uint8_t mPayloadLength);

    /**
     * This function sets the max payload length.Payload length in bytes.
     * The register needs to be set in implicit header mode for the expected packet length. (Means: In this mode the header is removed from the packet.)
     * A 0 value is not permitted
     * @param payll
     */
    void setPayloadLength(uint8_t payll);

    /**
     * Sets the frequencyHopping Period
     * Symbol periods between frequency hops. (0 = disabled). 1st hop always happen after the 1st header symbol
     * @param fhhp
     */
    void setFrequencyHoppingPeriod(uint8_t fhhp);


    /**
     * This function set the Gain of the output signal.
     * LNA gain setting: 000 not used
     * [Bits 7-5]
     * value 000 not used
     * value 001 G1 = maximum gain
     * 010 G2
     * 011 G3
     * 100 G4
     * 101 G5
     * 110 G6 = minimum gain
     * 111 not used
     * [Bits 4-3] Low Frequency (RFI_LF) LNA current adjustment 00 Default LNA current Other   Reserved
     * [bits 2] reserved for who knows
     * [bit 1-0] High Frequency (RFI_HF) LNA current adjustment
     * value 00 -> Default LNA current
     * value 11 -> Boost on, 150% LNA current
     * @param lnaMaxGain
     */
    void setLnaGain(uint8_t lnaMaxGain);

    /**
     * This function is the userfriendly way to init (set all registers) the RFM95 for receiving LoRa packages.
     * This function is based on the Single Channel LoRaWAN Gateway from Thomas Telkamp thomas@telkamp.eu . Thank you for his work.
     */
    void defaultLoRaSetup();

    void loraSetup(uint32_t fq, uint8_t sf , uint8_t cr);
    /**
     * writes zeros to the char buffer / array
     * @param arr
     */
    void clearCharBuffer(char * arr);

    /**
     * This function returns the interrupt pin on which the RFM95 instance has been created
     * @return -> privat _irq_pin
     */
    int getIRQpin();

    /**
     * This function returns the Reset pin on which the RFM95 instance has been created
     * @return -> _RST_pin
     */
    int getRSTpin();
    /**
     * This function returns the Chip select pin on which the RFM95 instance has been created
     * @return -> _cs_pin
     */
    int getCSpin();


    /**
     * This function sets the RFM95 registers so that the maximum FIFO data buffer size in transmit or receive mode can be used.
     * The LoRa Fifo of the RFM95 has a 256 byte RAM data buffer (called FiFo).
     * Important! If the Fifo is expanded the RFM95 can not Tx or Rx simultaneously automaticly.
     */
    void expandRFM95DataBuffToFullSize();

    /**
     * This function reads the latest received LoRa package from the FiFo (LoRa Data buffer)
     * @param arr -> it takes an byte array of max 256 byte to write the data to
     */
    void rxReceivedLoRaPackage(uint8_t * arr);


    /**
     * This function sets the spreading factor.
     * @param sf -> SFMode
     */
    void setSpredingFactor( uint8_t sf);


    /**
     * This function sets the coding rate
     * @param cr
     */
    void setCodingRate(uint8_t cr);

    void setModemConfigReg3();

    typedef enum {
        RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
        RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
        RHModeIdle,             ///< Transport is idle.
        RHModeTx,               ///< Transport is in the process of transmitting a message.
        RHModeRx                ///< Transport is in the process of receiving a message.
    } RHMode;

    typedef enum {
        SF6 = 6,
        SF7 , ///< Spreeading Factor 7. Initial default value
        SF8 ,
        SF9,
        SF10,
        SF11,
        SF12
    } SFMode;
private:
    /// Chipselect Pin
    volatile int _cs_pin;
    /// Pin where the interrupt will be reconized
    volatile int _irq_pin;
    /// Reset Pin where the RFM95 can be hard reseted
    volatile int _RST_pin;
    /// The current transport operating mode
    volatile RHMode _mode;
    ///    Frequency
    uint32_t _freq; // in Mhz! (868.1)
    ///  spreading factor (SF7 - SF12)
    uint8_t _sf;
    /// Coding rate
    uint8_t _cr;

    ///Bandwith
    uint8_t _bw;

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t _interruptCount;
    /// The configured interrupt pin connected to this instance
    uint8_t _interruptPin;
    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t _myInterruptIndex;
    /// Number of octets in the buffer
    volatile uint8_t _bufLen;
    /// The receiver/transmitter buffer
    uint8_t _buf[RH_RF95_MAX_PAYLOAD_LEN];
    /// True when there is a valid message in the buffer
    volatile bool _rxBufValid;
    /// The value of the last received RSSI value, in some transport specific units
    volatile int8_t _lastRssi = 0;
    /// Count of the number of bad messages (eg bad checksum etc) received
    volatile uint16_t _rxBad = 0;
    /// Count of the number of successfully transmitted messaged
    volatile uint16_t _rxGood;
    /// Count of the number of bad messages (correct checksum etc) received
    volatile uint16_t _txGood;

};


#endif //HOPERF_LORA_RECEIVER_LABB_RFM95_H
