//
// Created by Georg Rokita on 03.11.16.
//

#ifndef HOPERF_LORA_RECEIVER_LABB_RFM95_H
#define HOPERF_LORA_RECEIVER_LABB_RFM95_H


#include <cstdint>
#include "RFM95registers.h"


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
     * This function sets the connected SPI Chipselcet pin to LOW to selctet the chip for SPI communication
     */
    void selectreceiver();

    /**
    * This function sets the connected SPI Chipselcet pin to HIGH to unselctet the chip for SPI communication.
    */
    void unselectreceiver();

    typedef enum {
        RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
        RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
        RHModeIdle,             ///< Transport is idle.
        RHModeTx,               ///< Transport is in the process of transmitting a message.
        RHModeRx                ///< Transport is in the process of receiving a message.
    } RHMode;

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
    uint32_t _freq ; // in Mhz! (868.1)
    /// Set spreading factor (SF7 - SF12)
    uint8_t _sf;

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
