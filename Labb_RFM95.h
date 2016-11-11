//
// Created by Georg Rokita on 03.11.16.
// This Class includes code from Thomas Telkamp thomas@telkamp.eu and the Single Channel LoRaWAN Gateway and also the Radio Head Library RFM95
// It implies the following dependencies
// SPI  enabled on the Raspberry Pi
// WiringPi: a GPIO access library written in C for the BCM2835 used in the Raspberry Pi. sudo apt-get install wiringpi see http://wiringpi.com
//

#ifndef HOPERF_LORA_RECEIVER_LABB_RFM95_H
#define HOPERF_LORA_RECEIVER_LABB_RFM95_H

#include <cstdint>

#include "AES.h"

#define CHANNEL 0

#define RF95_SYMB_TIMEOUT           0x64    /// 0x08
#define RF95_MAX_PAYLOAD_LENGTH     0xff    /// default accepted payload length -> all packages can be received
#define PAYLOAD_LENGTH              0x40
#define FREQ_HOP_PERIOD             0x00    /// 0x00 means freq hopping is turned off
#define LNA_MAX_GAIN                0x23    ///define LNA_MAX_GAIN //LowNoiseAmplifier Gain if the gateway is not set in automatic gain controll

#define RX_ENCRYPTED_BUFFER         16

#define SX1276_MODE_Continuos       0x85

#define CR_4_5 5
#define CR_4_6 6
#define CR_4_7 7
#define CR_4_8 8

#define RH_RF95_FIFO_SIZE           255     /// Max number of octets the LORA Rx/Tx FIFO can hold


#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE   /// This is the maximum number of bytes that can be carried by the LORA.
                                                    /// We use some for headers, keeping fewer for RadioHead messages

#define RH_RF95_FXOSC 32000000.0            /// The crystal oscillator frequency of the module

//############## Copied from Radio Head Library (OpenSource) / RFM95.h ########
// Register names (LoRa Mode, from table 85)


#define RH_RF95_REG_00_FIFO                                0x00
#define RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_REG_02_RESERVED                            0x02
#define RH_RF95_REG_03_RESERVED                            0x03
#define RH_RF95_REG_04_RESERVED                            0x04
#define RH_RF95_REG_05_RESERVED                            0x05
#define RH_RF95_REG_06_FRF_MSB                             0x06
#define RH_RF95_REG_07_FRF_MID                             0x07
#define RH_RF95_REG_08_FRF_LSB                             0x08
#define RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RH_RF95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_REG_13_RX_NB_BYTES                         0x13
#define RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_REG_19_PKT_SNR_VALUE                       0x19
#define RH_RF95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RH_RF95_REG_1B_RSSI_VALUE                          0x1b
#define RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RH_RF95_REG_20_PREAMBLE_MSB                        0x20
#define RH_RF95_REG_21_PREAMBLE_LSB                        0x21
#define RH_RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RH_RF95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RH_RF95_REG_24_HOP_PERIOD                          0x24
#define RH_RF95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RH_RF95_REG_26_MODEM_CONFIG3                       0x26

#define RH_RF95_REG_40_DIO_MAPPING1                        0x40
#define RH_RF95_REG_41_DIO_MAPPING2                        0x41
#define RH_RF95_REG_42_VERSION                             0x42

#define RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_REG_5B_FORMER_TEMP                         0x5b
#define RH_RF95_REG_61_AGC_REF                             0x61
#define RH_RF95_REG_62_AGC_THRESH1                         0x62
#define RH_RF95_REG_63_AGC_THRESH2                         0x63
#define RH_RF95_REG_64_AGC_THRESH3                         0x64

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE                       0x80
#define RH_RF95_ACCESS_SHARED_REG                     0x40
#define RH_RF95_MODE                                  0x07
#define RH_RF95_MODE_SLEEP                            0x00
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_MODE_FSTX                             0x02
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_FSRX                             0x04
#define RH_RF95_MODE_RXCONTINUOUS                     0x05
#define RH_RF95_MODE_RXSINGLE                         0x06
#define RH_RF95_MODE_CAD                              0x07

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_OUTPUT_POWER                          0x0f

// RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RH_RF95_PA_RAMP                               0x0f
#define RH_RF95_PA_RAMP_3_4MS                         0x00
#define RH_RF95_PA_RAMP_2MS                           0x01
#define RH_RF95_PA_RAMP_1MS                           0x02
#define RH_RF95_PA_RAMP_500US                         0x03
#define RH_RF95_PA_RAMP_250US                         0x0
#define RH_RF95_PA_RAMP_125US                         0x05
#define RH_RF95_PA_RAMP_100US                         0x06
#define RH_RF95_PA_RAMP_62US                          0x07
#define RH_RF95_PA_RAMP_50US                          0x08
#define RH_RF95_PA_RAMP_40US                          0x09
#define RH_RF95_PA_RAMP_31US                          0x0a
#define RH_RF95_PA_RAMP_25US                          0x0b
#define RH_RF95_PA_RAMP_20US                          0x0c
#define RH_RF95_PA_RAMP_15US                          0x0d
#define RH_RF95_PA_RAMP_12US                          0x0e
#define RH_RF95_PA_RAMP_10US                          0x0f

// RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_OCP_ON                                0x20
#define RH_RF95_OCP_TRIM                              0x1f

// RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_LNA_GAIN                              0xe0
#define RH_RF95_LNA_BOOST                             0x03
#define RH_RF95_LNA_BOOST_DEFAULT                     0x00
#define RH_RF95_LNA_BOOST_150PC                       0x11

// RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_RX_TIMEOUT_MASK                       0x80
#define RH_RF95_RX_DONE_MASK                          0x40
#define RH_RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RH_RF95_VALID_HEADER_MASK                     0x10
#define RH_RF95_TX_DONE_MASK                          0x08
#define RH_RF95_CAD_DONE_MASK                         0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RH_RF95_CAD_DETECTED_MASK                     0x01

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_RX_TIMEOUT                            0x80
#define RH_RF95_PACKET_RECEPTION_COMPLETE			  0x50
#define RH_RF95_RX_DONE                               0x40
#define RH_RF95_PAYLOAD_CRC_ERROR                     0x20
#define RH_RF95_VALID_HEADER                          0x10
#define RH_RF95_TX_DONE                               0x08
#define RH_RF95_CAD_DONE                              0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RH_RF95_CAD_DETECTED                          0x01

// RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_RX_CODING_RATE                        0xe0
#define RH_RF95_MODEM_STATUS_CLEAR                    0x10
#define RH_RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RH_RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RH_RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_PLL_TIMEOUT                           0x80
#define RH_RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RH_RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_BW                                    0xc0
#define RH_RF95_BW_125KHZ                             0x00
#define RH_RF95_BW_250KHZ                             0x40
#define RH_RF95_BW_500KHZ                             0x80
#define RH_RF95_BW_RESERVED                           0xc0
#define RH_RF95_CODING_RATE                           0x38
#define RH_RF95_CODING_RATE_4_5                       0x00
#define RH_RF95_CODING_RATE_4_6                       0x08
#define RH_RF95_CODING_RATE_4_7                       0x10
#define RH_RF95_CODING_RATE_4_8                       0x18
#define RH_RF95_IMPLICIT_HEADER_MODE_ON               0x04
#define RH_RF95_RX_PAYLOAD_CRC_ON                     0x02
#define RH_RF95_LOW_DATA_RATE_OPTIMIZE                0x01

// RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_SPREADING_FACTOR                      0xf0
#define RH_RF95_SPREADING_FACTOR_64CPS                0x60
#define RH_RF95_SPREADING_FACTOR_128CPS               0x70
#define RH_RF95_SPREADING_FACTOR_256CPS               0x80
#define RH_RF95_SPREADING_FACTOR_512CPS               0x90
#define RH_RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RH_RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RH_RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RH_RF95_TX_CONTINUOUS_MOE                     0x08
#define RH_RF95_AGC_AUTO_ON                           0x04
#define RH_RF95_SYM_TIMEOUT_MSB                       0x03

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_DAC_ENABLE                         0x07



// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)

//###########################################


class Labb_RFM95: public AES {

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

    void loraSetup(uint32_t fq, uint8_t sf, uint8_t cr);

    /**
     * writes zeros to the char buffer / array
     * @param arr
     */
    void clearCharBuffer(char *arr);

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
    void rxReceivedLoRaPackage(uint8_t *arr);


    /**
     * This function sets the spreading factor.
     * @param sf -> SFMode
     */
    void setSpredingFactor(uint8_t sf);


    /**
     * This function sets the coding rate
     * @param cr
     */
    void setCodingRate(uint8_t cr);

    /** This function sets register modem config 3.
     * It needs to be called when setting up frequency, spreading factor and coding rate.
     * Speaking in generall all three config Modem registers needs to be set up.
     */
    void setModemConfigReg3();

    /**
     * This function should be used, when the programm should be started from the command line with <options>.
     * Valide <options> -f <freq value in Hz> -sf <spreading as number> -cr <coding rate as number>
     * e.g. ( -f 868100 -sf 7 -cr 5 )
     * It will check the option parameters and call the loraSetup(uint32_t fq, uint8_t sf , uint8_t cr) to init the RFM95.
     * @param argcount -> number of arguments in the argvector
     * @param argvector -> options array / vector
     */
    bool checkCommandLineArgLoraSetup(int argcount, char *argvector[]);


    /**
     * This function copies the given byte array into a char array
     * @param arr -> takes the byte buffer / array, which was read from the RFM95 FiFo
     * @return -> pointer to the filled up char array
     */
    char * convertByteBufToCharBuf(uint8_t *arr, int bufLen);

    /**
     * This function prints an char array to the screen
     * @param arr
     * @param bufLen
     */
    void printCharBuffer(const char *arr, int bufLen);


    /**
     * This function returns the number of received Bytes
     * @return uint8-t number of received bytes
     */
    uint8_t getBufLen();


    uint8_t * shiftBuf(uint8_t *arr,const int bufLen);

    /**
     * prints all Bytes in the rx Buffer
     */

    void printOutByteBuf();


    /**
     *  Getter method for _debug
     * @return _debug status
     */
    bool is_debug() const;
    /**
     * Setter method for _debug
     * @param _debug
     */
    void set_debug(bool _debug);

    /**
     * User-friendly main function
     * This function has the routine implemted all steps from the detected interrupt to convertion of the binear data into char Data.
     * The routine is the following.
     * 1. check if LoRa package receive by flags are set and read the FiFo Buffer -> handleInterrupt()
     * 2. divide the Header from the Payload
     * 3. decrypted the Payload
     * 4. convert the Payload into characters
     * 5. print the Payload in characters
     * 6. clear buffers
     *
     * This used to be my main function while developing the Labb_RFM95 class.
     * By reading this function get know the class and how it works and can be used.
     *
     */

    void mainLoRaHandler();

    /**
     * This function is a debug helper to print the different buffers
     * @param str -> A string to identify the printed buffer
     * @param arr -> buffer which should be printed
     * @param bytes -> length of the arr
     */
    void print_value (char * str, uint8_t * arr, int bytes);


    /**
     * This function decryptes the payload.
     * @param cipher
     * @return
     */
    uint8_t * decrypData(uint8_t cipher[]);

    bool is_palyoadEncryp() const;

    void set_palyoadEncryp(bool _palyoadEncryp);


    typedef enum {
        RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
        RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
        RHModeIdle,             ///< Transport is idle.
        RHModeTx,               ///< Transport is in the process of transmitting a message.
        RHModeRx                ///< Transport is in the process of receiving a message.
    } RHMode;

    typedef enum {
        SF6 = 6,
        SF7, ///< Spreeading Factor 7. Initial default value
        SF8,
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
    /// Pointer to array where the decrypted payload of the LoRa Phy Payload is stored
    uint8_t * _decryDataArrPtr;
    /// Pointer to array where the converted Data is stored
    char * _charArrPtr;
    /// Pointer to array where the encypted payload of the LoRa Phy Payload is stored
    uint8_t * _payloadDataArrptr;
    /// Debugging Mode -> print the registers
    bool _debug;
    /// If Payload is encrypted or not
    bool _palyoadEncryp;

    /**
     * This funtion is a helper function for the  checkCommandLineArgLoraSetup().
     * It prints out the possible input <options> on the command line.
    */
    void explainUsage();
};


#endif //HOPERF_LORA_RECEIVER_LABB_RFM95_H
