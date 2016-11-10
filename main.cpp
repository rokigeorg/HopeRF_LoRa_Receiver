#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <errno.h>
#include <sys/types.h>

#include <string.h>
#include <string>
#include <cstring>
#include <stdint.h>
#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "AES.h"

#include "RFM95registers.h"
#include "Labb_RFM95.h"


using namespace std;

// create AES instance
AES aes;



void print_value (char * str, uint8_t * a, int bytes)
{
    printf ("%s ", str) ;

    int end =bytes;
    for (int i = 0 ; i < end ; i++)
    {
        printf ("%x",a[i]) ;
    }
    printf ("\n") ;
}

uint8_t * decrypData(uint8_t cipher[])
{
    uint8_t key[] = "0123456789987654";
    static uint8_t plain [N_BLOCK] ;
    uint8_t bits = 128;

    uint8_t succ ;

    /// Set key
    succ = aes.set_key (key, bits) ;
    if (succ != SUCCESS)
         cout <<"Failure set_key" << endl ;

    succ = aes.decrypt (cipher, plain) ;
    if (succ != SUCCESS)
        cout <<"Failure encrypt" << endl ;

    return plain;
}

int main(int argc, char* argv[]) {
    ///create instance of the Labb_RFM95 class
    Labb_RFM95 labb_rfm95(6,7,0);

    ///wait until RF95 is resetted
    while(!labb_rfm95.resetRFM95());

    cout << "Tries to start the RFM95!" << endl;
    ///check the command line args and start the RFM95 Modul in LoRa Mode continues receive Mode
    if(labb_rfm95.checkCommandLineArgLoraSetup(argc, argv) == false)
        return 1;

    cout << "RFM95 is setup in LoRa continues receive mode!" << endl;
    cout <<"******************"<< endl;
    labb_rfm95.printAllRegisters();

    uint8_t * decryDataArrPtr;
    char * charArrPtr;
    uint8_t byteBuffer[RH_RF95_MAX_PAYLOAD_LEN];
    uint8_t * encrypDataArrptr;

    int bufLen = RH_RF95_MAX_PAYLOAD_LEN;

    while(1){

        //busy waiting, when Pin 0 goes HIGH it contiuous
        //while(digitalRead(dio0));
        //check if interrupt flag has been set
        //RFM95 Modul sets DIO0 pin (check pinlayout on the breakout board [Adafruit RFM9x -> D]to high when message arrives

        if(digitalRead(7) == true)//labb_rfm95.getIRQpin()
        {


            if(labb_rfm95.readRegister(RH_RF95_REG_12_IRQ_FLAGS) == RH_RF95_PACKET_RECEPTION_COMPLETE){
                printf("\n");
                printf("************************************\n");
                printf("Mode: %x\n", labb_rfm95.readRegister(RH_RF95_REG_01_OP_MODE) );
                printf("Interrupt Register: %x\n", labb_rfm95.readRegister(RH_RF95_REG_12_IRQ_FLAGS));
                printf("\n");
                printf("Byte Addr of the last writen Rx Byte: %x\n", labb_rfm95.readRegister(RH_RF95_REG_25_FIFO_RX_BYTE_ADDR));
                printf("Received Number of Bytes: 0x%x dec:%d \n", labb_rfm95.readRegister(RH_RF95_REG_13_RX_NB_BYTES),labb_rfm95.readRegister(RH_RF95_REG_13_RX_NB_BYTES));
                printf("FiFo Current Rx Addr: %x\n", labb_rfm95.readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
                printf("FiFo Addr Ptr: %x\n", labb_rfm95.readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));

            }
            labb_rfm95.handleInterrupt();
            labb_rfm95.rxReceivedLoRaPackage(byteBuffer);

            ///print out the received bytes / byteBuf
            print_value ((char *) "ByteBuffer: ", byteBuffer, (int) labb_rfm95.getBufLen());

            /// cut of the 2Byte sinnlos am anfang & 4bytes sinnlos am ende
            encrypDataArrptr = labb_rfm95.shiftBuf(byteBuffer,(int) labb_rfm95.getBufLen());

            ///print out the received bytes / byteBuf
            print_value ((char *) "encrypDataBuffer: ", encrypDataArrptr, RX_ENCRYPTED_BUFFER);

            ///decrypt the received Bytes
            decryDataArrPtr = decrypData(encrypDataArrptr);

            //print buffer
            printf("Buffer:\n");

            //charArrPtr = labb_rfm95.convertByteBufToCharBuf(byteBuffer, bufLen);
            charArrPtr = labb_rfm95.convertByteBufToCharBuf(decryDataArrPtr, (int) labb_rfm95.getBufLen());
            //labb_rfm95.printCharBuffer(charArrPtr, bufLen);
            labb_rfm95.printCharBuffer(charArrPtr, (int) labb_rfm95.getBufLen());

            ///fill the charBuffer to all 0
            labb_rfm95.clearCharBuffer(charArrPtr);


            memset(byteBuffer, 0, sizeof byteBuffer);
        }
    }

    return 0;
}


