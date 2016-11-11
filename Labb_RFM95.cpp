//
// Created by Georg Rokita on 03.11.16.
//

#include <iostream>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "Labb_RFM95.h"

#define RF95_FREQ 868100000 //868.1 MHz
#define RF95_SF 7


template <typename T, size_t N>
inline
size_t SizeOfArray( const T(&)[ N ] )
{
    return N;
}

Labb_RFM95::Labb_RFM95(int cs_pin, int irq_pin, int RST_pin):AES() {
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
    _freq = 868100000;      ///// 868.1 MHz
    _sf = 7;                /// SF 6 64 chips/symbol; SF 7 128 chips/symbol (default); SF 8 256 chips/symbol; SF 9 512 chips/symbol; SF 10 1024 chips/symbol; SF 11 2048 chips/symbol; SF 12 4096 chips/symbol
    _bw = 0x07;             /// default Bandwidth 125.0 kHZ
    _debug = true;
    _palyoadEncryp = true;  /// if payload is encrypted or not

}

bool Labb_RFM95::is_palyoadEncryp() const {
    return _palyoadEncryp;
}

void Labb_RFM95::set_palyoadEncryp(bool _palyoadEncryp) {
    Labb_RFM95::_palyoadEncryp = _palyoadEncryp;
}


Labb_RFM95::~Labb_RFM95() {
}

bool Labb_RFM95::resetRFM95(){

    digitalWrite(_RST_pin, LOW);
    delay(100);
    digitalWrite(_RST_pin, HIGH);
    delay(100);
    return true;
}

bool Labb_RFM95::is_debug() const {
    return _debug;
}

void Labb_RFM95::set_debug(bool _debug) {
    Labb_RFM95::_debug = _debug;
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
    uint8_t registers[] = {RH_RF95_REG_00_FIFO, RH_RF95_REG_01_OP_MODE, RH_RF95_REG_02_RESERVED, RH_RF95_REG_03_RESERVED, RH_RF95_REG_04_RESERVED, RH_RF95_REG_05_RESERVED, RH_RF95_REG_06_FRF_MSB, RH_RF95_REG_07_FRF_MID, RH_RF95_REG_08_FRF_LSB , RH_RF95_REG_09_PA_CONFIG, RH_RF95_REG_0A_PA_RAMP, RH_RF95_REG_0B_OCP , RH_RF95_REG_0C_LNA ,
                           RH_RF95_REG_0D_FIFO_ADDR_PTR, RH_RF95_REG_0E_FIFO_TX_BASE_ADDR,RH_RF95_REG_0F_FIFO_RX_BASE_ADDR , RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, RH_RF95_REG_11_IRQ_FLAGS_MASK,RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_REG_13_RX_NB_BYTES, RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB,
                           RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB, RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB, RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB,RH_RF95_REG_18_MODEM_STAT,
                           RH_RF95_REG_19_PKT_SNR_VALUE,RH_RF95_REG_1A_PKT_RSSI_VALUE,RH_RF95_REG_1B_RSSI_VALUE,RH_RF95_REG_1C_HOP_CHANNEL,RH_RF95_REG_1D_MODEM_CONFIG1, RH_RF95_REG_1E_MODEM_CONFIG2, RH_RF95_REG_1F_SYMB_TIMEOUT_LSB, RH_RF95_REG_20_PREAMBLE_MSB ,RH_RF95_REG_21_PREAMBLE_LSB, RH_RF95_REG_22_PAYLOAD_LENGTH, RH_RF95_REG_23_MAX_PAYLOAD_LENGTH ,RH_RF95_REG_24_HOP_PERIOD, RH_RF95_REG_25_FIFO_RX_BYTE_ADDR,RH_RF95_REG_26_MODEM_CONFIG3};

    std::string registerNames[] = {"RH_RF95_REG_00_FIFO","RH_RF95_REG_01_OP_MODE", "RH_RF95_REG_02_RESERVED", "RH_RF95_REG_03_RESERVED", "RH_RF95_REG_04_RESERVED", "RH_RF95_REG_05_RESERVED", "RH_RF95_REG_06_FRF_MSB", "RH_RF95_REG_07_FRF_MID", "RH_RF95_REG_08_FRF_LSB" , "RH_RF95_REG_09_PA_CONFIG", "RH_RF95_REG_0A_PA_RAMP", "RH_RF95_REG_0B_OCP", "RH_RF95_REG_0C_LNA" ,
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
        //printf("\n FiFo Addr Ptr: %x\n", readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));

        spiBurstRead(_buf, len);
        _bufLen = len;
        writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

        // Remember the RSSI of this packet
        // this is according to the doc, but is it really correct?
        // weakest receiveable signals are reported RSSI at about -66
        _lastRssi = readRegister(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;
    }
}

bool Labb_RFM95::setFrequency(uint32_t freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(RH_RF95_REG_06_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(RH_RF95_REG_07_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(RH_RF95_REG_08_FRF_LSB, (uint8_t)(frf>> 0) );
    return true;
}

void Labb_RFM95::setModemRegisters(){
    writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, 0x72);
    writeRegister(RH_RF95_REG_1E_MODEM_CONFIG2, (_sf<<4) | 0x04);
    writeRegister(RH_RF95_REG_26_MODEM_CONFIG3, 0x04);  ///[7-4 bit: unused][3 bit: 0->static node / 1->mobile node] [2 bit: 0->LNA gain set by register LnaGain / 1->LNA gain set by the internal AGC loop][1-0 bit: reserved]
}

void Labb_RFM95::setSymbTimeout(uint8_t timeOutPeriod){
    writeRegister(RH_RF95_REG_1F_SYMB_TIMEOUT_LSB,   timeOutPeriod);
}

void Labb_RFM95::setMaxPayloadLength(uint8_t mPayloadLength){
    writeRegister(RH_RF95_REG_23_MAX_PAYLOAD_LENGTH,   mPayloadLength);
}

void Labb_RFM95::setPayloadLength(uint8_t payll) {
    writeRegister(RH_RF95_REG_22_PAYLOAD_LENGTH,  payll);
}

void Labb_RFM95::setFrequencyHoppingPeriod(uint8_t fhhp){
    writeRegister(RH_RF95_REG_24_HOP_PERIOD,fhhp);
}

void Labb_RFM95::setLnaGain(uint8_t lnaMaxGain){
    writeRegister(RH_RF95_REG_0C_LNA, LNA_MAX_GAIN);  // max lna gain
}

void Labb_RFM95::clearCharBuffer(char * arr){
    int i = 0;
    while (arr[i]== '\n'){
        arr[i]=0;
        i++;
    }
}


void Labb_RFM95::defaultLoRaSetup()
{
    //wait until RF95 is resetted
    while(!resetRFM95());
    printf("SX1276 detected, starting.\n");

    uint8_t version = readRegister(RH_RF95_REG_42_VERSION);
    if (version == 0x12) {
        // sx1276
        printf("SX1276 detected, starting.\n");
        printf("Version: 0x%x\n",version);
    } else {
        printf("Unrecognized transceiver.\n");
        printf("Version: 0x%x\n",version);
        // exit(1);
    }

    // Set Continous Sleep Mode
    writeRegister(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE);
    printf("Set in LONG_RANGE_MODE. RH_RF95_REG_01_OP_MODE value: %x \n", readRegister(RH_RF95_REG_01_OP_MODE));

    //set Frequency to 868.1 MHz by default
    printf("Set frequency to: %d Hz\n", _freq);
    setFrequency(_freq);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequency");}

    setModemRegisters();
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setModemRegisters");}

    ///setTimeout RX operation time-out value expressed as number of symbols:
    setSymbTimeout(RF95_SYMB_TIMEOUT);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setSymbTimeout");}
    //set Max Payload length to filter for the right packetes
    setMaxPayloadLength(RF95_MAX_PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setMaxPayloadLength");}

    setPayloadLength(PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setPayloadLength");}

    setFrequencyHoppingPeriod(FREQ_HOP_PERIOD);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequencyHoppingPeriod");}

    writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, readRegister(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR));

    // Set Continous Receive Mode
    writeRegister(RH_RF95_REG_01_OP_MODE, SX1276_MODE_Continuos);
}

int Labb_RFM95::getIRQpin() {
    return _irq_pin;
}

int Labb_RFM95::getRSTpin() {
    return _RST_pin;
}

int Labb_RFM95::getCSpin() {
    return _cs_pin;
}

void Labb_RFM95::expandRFM95DataBuffToFullSize() {
    writeRegister(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR,0x00);
    writeRegister(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR,0x00);
}

void Labb_RFM95::rxReceivedLoRaPackage(uint8_t *arr) {

    /// The actual location to be read from, or written to, over the SPI interface is defined by the address pointer FifoAddrPtr.
    /// Before any read or write operation it is hence necessary to initialise this pointer to the corresponding base value.
    /// Upon reading or writing to the FIFO data buffer (RegFifo) the address pointer will then increment automatically.
    /// The register FifoRxCurrentAddr indicates the location of the last packet received in the FIFO so that
    /// the last packet received can be easily read by pointing the FifoAddrPtr to this register.

    uint8_t fiFo_Addr = readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);
    fiFo_Addr = (uint8_t) (fiFo_Addr + 0x02);
    writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, fiFo_Addr);

    // Have received a packet
    uint8_t receivedCount = readRegister(RH_RF95_REG_13_RX_NB_BYTES);     //read register which tells the Number of received bytes
    uint8_t receivedbytes = (uint8_t) (receivedCount - 2);

    for(int i = 0; i < receivedbytes; i++)
    {
        arr[i] = readRegister(RH_RF95_REG_00_FIFO);
    }

}

void Labb_RFM95::loraSetup(uint32_t fq, uint8_t sf, uint8_t cr) {
    _freq = fq;
    _sf = sf;

    //wait until RF95 is resetted
    while(!resetRFM95());
    printf("SX1276 detected, starting.\n");

    uint8_t version = readRegister(RH_RF95_REG_42_VERSION);
    if (version == 0x12) {
        // sx1276
        printf("SX1276 detected, starting.\n");
        printf("Version: 0x%x\n",version);
    } else {
        printf("Unrecognized transceiver.\n");
        printf("Version: 0x%x\n",version);
        // exit(1);
    }

    /// Set LONG_RANGE_MODE Mode
    writeRegister(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE);

    ///set up freq on the RFM95
    setFrequency(_freq);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequency");}

    ///set up spreading factor on the RFM95
    setSpredingFactor(sf);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setSpredingFactor");}

    /// set coding rate
    setCodingRate(cr);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setCodingRate");}

    /// set MODEM CONFIG 3 Register
    setModemConfigReg3();
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setModemConfigReg3");}

    ///setTimeout RX operation time-out value expressed as number of symbols:
    setSymbTimeout(RF95_SYMB_TIMEOUT);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setSymbTimeout");}
    ///set Max Payload length to filter for the right packetes
    setMaxPayloadLength(RF95_MAX_PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setMaxPayloadLength");}

    setPayloadLength(PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setPayloadLength");}

    setFrequencyHoppingPeriod(FREQ_HOP_PERIOD);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequencyHoppingPeriod");}

    writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, readRegister(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR));

    // Set Continous Receive Mode
    writeRegister(RH_RF95_REG_01_OP_MODE, SX1276_MODE_Continuos);
}


void Labb_RFM95::setSpredingFactor(uint8_t sf) {
    _sf = sf;
    writeRegister(RH_RF95_REG_1E_MODEM_CONFIG2, (_sf<<4) | 0x04);
}

void Labb_RFM95::setCodingRate(uint8_t cr) {
    _cr = cr;

    if(cr == CR_4_5) {
        writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, (0x72));
    }
    else if(cr == CR_4_6) {
        writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, (0x74));
    }
    else if(cr == CR_4_7) {
        writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, (0x76));
    }
    else if(cr == CR_4_8) {
        writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, (0x78));
    }
    else {
        std::cout << "Coding Rate paramater is not valide. Please enter 5 - 8 for '4/5' - '4/8'" <<std::endl;
    }
}

void Labb_RFM95::setModemConfigReg3() {

    writeRegister(RH_RF95_REG_26_MODEM_CONFIG3, 0x04);  ///[7-4 bit: unused][3 bit: 0->static node / 1->mobile node] [2 bit: 0->LNA gain set by register LnaGain / 1->LNA gain set by the internal AGC loop][1-0 bit: reserved]

}

bool Labb_RFM95::checkCommandLineArgLoraSetup(int argcount, char **argvector) {

    if (argcount < 3) {
        explainUsage();
        return false;
    }

    int argVal;
    uint32_t freq;
    uint8_t sf =7 , cr = 5;

    for (int i = 1; i < argcount; ++i) {
        if (std::string(argvector[i]) == "-f") {
            if (i + 1 < argcount) { // Make sure we aren't at the end of argv!
                argVal =  atoi(argvector[(i+1)]); // Increment 'i' so we don't get the argument as the next argv[i].
                freq = (uint32_t) argVal;
            }
        } else if(std::string(argvector[i]) == "-sf"){
            if (i + 1 < argcount) {
                argVal = atoi(argvector[(i+1)]);
                sf = (uint8_t) argVal;
            }
        }
        else if(std::string(argvector[i]) == "-cr"){
            if (i + 1 < argcount) {
                argVal = atoi(argvector[(i+1)]);
                cr = (uint8_t) argVal;
            }
        }
        else if(std::string(argvector[i]) == "-h"){
            explainUsage();
        }
    }

    ///setup RFM95 Modul into continues receiving mode
    loraSetup(freq, sf, cr);
    return true;
}

void Labb_RFM95::explainUsage() {
    std::cerr << "Usage: <option(s)> SOURCES"
              << "Options:\n"
              << "\t-h\t\tShow this help message\n"
              << "\t-f\t\tfor entering frequency in Hz. After -f <freq-value>\n"
              << "\t-sf\t\t for spreading factor as number. After -sf <spreading factor-value>  e.g. SF7 = 7\n"
              << "\t-cr\t\t for coding rate as number. After -cr <coding rate-value>  e.g. 4/5 = 5\n" <<std::endl;

}

char *Labb_RFM95::convertByteBufToCharBuf(uint8_t *arr, int bufLen) {

    static char charBuffer[RH_RF95_MAX_PAYLOAD_LEN];

    for(int i=0; i < bufLen;i++){
        charBuffer[i] = (char) arr[i];
    }

    return charBuffer;
}

void Labb_RFM95::printCharBuffer(const char *arr, int bufLen) {

    int i=0;
    for(i=0; i < bufLen;i++){
        printf("%c", arr[i]);
    }
}

uint8_t Labb_RFM95::getBufLen() {
    return _bufLen;
}

uint8_t *Labb_RFM95::shiftBuf(uint8_t *arr,const int bufLen) {

    //static uint8_t shiftedBuf[RH_RF95_MAX_PAYLOAD_LEN];

    static uint8_t shiftedBuf[(RX_ENCRYPTED_BUFFER + 1)];

    int endData = bufLen;

    int j = 0;
    for (int i = 4; i < endData; i++) {

        shiftedBuf[j] = arr[i];
        ++j;
    }

    return shiftedBuf;
}


void Labb_RFM95::printOutByteBuf() {

    std::cout << "private Byte Buffer: ";

    for (int i = 0 ; i < RH_RF95_MAX_PAYLOAD_LEN ; i++)
    {
        printf ("%x",_buf[i]) ;
    }
    printf ("\n") ;
}

void Labb_RFM95::mainLoRaHandler() {

    if(digitalRead(_irq_pin) == true) /// check if interrupt has happend
    {
        if(_debug){
            if(readRegister(RH_RF95_REG_12_IRQ_FLAGS) == RH_RF95_PACKET_RECEPTION_COMPLETE){
                printf("\n");
                printf("************************************\n");
                printf("Mode: %x\n", readRegister(RH_RF95_REG_01_OP_MODE) );
                printf("Interrupt Register: %x\n", readRegister(RH_RF95_REG_12_IRQ_FLAGS));
                printf("\n");
                printf("Byte Addr of the last writen Rx Byte: %x\n", readRegister(RH_RF95_REG_25_FIFO_RX_BYTE_ADDR));
                printf("Received Number of Bytes: 0x%x dec:%d \n", readRegister(RH_RF95_REG_13_RX_NB_BYTES),readRegister(RH_RF95_REG_13_RX_NB_BYTES));
                printf("FiFo Current Rx Addr: %x\n", readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
                printf("FiFo Addr Ptr: %x\n", readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));
            }
        }

        handleInterrupt();

        if(_debug){
            ///print out the received bytes / byteBuf
            print_value ((char *) "Header + Payload as Byte Buffer: ", _buf, (int) _bufLen);
        }

        /// cut of the 2Byte sinnlos am anfang & 4bytes sinnlos am ende
        _payloadDataArrptr = shiftBuf(_buf,(int) _bufLen);

        if(_debug){
            ///print out the received bytes / byteBuf
            print_value ((char *) "encrypDataBuffer: ", _payloadDataArrptr, RX_ENCRYPTED_BUFFER);
        }

        //print buffer
        std::cout <<"Buffer: "<< std::endl;

        if(_palyoadEncryp){
            ///decrypt the received Bytes
            _decryDataArrPtr = decrypData(_payloadDataArrptr);
            /// convert the payload into chars
            _charArrPtr = convertByteBufToCharBuf(_decryDataArrPtr, (int) _bufLen);

        } else{
            /// convert the unencrypted payload into chars
            _charArrPtr = convertByteBufToCharBuf(_payloadDataArrptr, _bufLen);
        }
        //labb_rfm95.printCharBuffer(charArrPtr, bufLen);
        printCharBuffer(_charArrPtr, (int) _bufLen);

        ///fill the charBuffer to all 0
        clearCharBuffer(_charArrPtr);

        memset(_buf, 0, sizeof _buf);
    }

}

void Labb_RFM95::print_value(char *str, uint8_t *arr, int bytes) {
    printf ("%s ", str) ;

    for (int i = 0 ; i < bytes ; i++)
    {
        printf ("%x",arr[i]) ;
    }
    printf ("\n") ;
}

uint8_t *Labb_RFM95::decrypData(uint8_t *cipher) {
    uint8_t key[] = "0123456789987654";
    static uint8_t plain [N_BLOCK] ;
    uint8_t bits = 128;

    uint8_t succ ;

    /// Set key
    succ = set_key(key, bits) ;
    if (succ != SUCCESS)
        std::cout <<"Failure set_key" << std::endl ;

    succ = decrypt (cipher, plain) ;
    if (succ != SUCCESS)
        std::cout <<"Failure encrypt" << std::endl ;

    return plain;
}
