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
/*

uint8_t * decrypData(uint8_t cipher[], int sizeOfPayload)
{
    unsigned char key[] = "0123456789010123";
    unsigned long long int my_iv = 36753562;

    // arr to hold the IV (i... vector), plain_p ???,  cipher( encrypted data), check (decrypted data)
    uint8_t iv [N_BLOCK] = {15420848200001542084820000};
    uint8_t plain_p[sizeOfPayload+7];
    //uint8_t cipher[(sizeOfPayload+7)];
    uint8_t check [sizeOfPayload+7] ;
    int bits = 128;

    //This function increased the VI by one step in order to have a different IV each time
    //aes.iv_inc();

    //aes.set_IV(my_iv);  // Sets IV (initialization vector) and IVC (IV counter). This function changes the ivc and iv variables needed for AES.
    //aes.get_IV(iv);     //This function return the IV @param out byte pointer that gets the IV. @return none but the IV is writed to the out pointer.

    cout << "IV: " ;
    for(int i =0; i<N_BLOCK;i++){
        printf("%d",iv[i]);
    }
    cout <<"\n";
    aes.do_aes_decrypt(cipher,sizeOfPayload,check,key,bits,iv); // encrypt (AES) data in plain[] and write it into cipher[]

    return check;
}
*/


uint8_t * decrypData(uint8_t cipher[], int sizeOfPayload)
{
    uint8_t key[] = "01234567899876543210012345678998";
    unsigned long long int my_iv = 36753562;

    // arr to hold the IV (i... vector), plain_p (data after decryption),  cipher( encrypted data), check (decrypted data)
    uint8_t iv [N_BLOCK];
    uint8_t plain_p[sizeOfPayload+7];
    //byte cipher[(sizeOfPayload+7)] ;
    uint8_t check [sizeOfPayload+7] ;
    int bits = 128;

    //This function increased the VI by one step in order to have a different IV each time
    aes.iv_inc();

    aes.set_IV(my_iv);  // Sets IV (initialization vector) and IVC (IV counter). This function changes the ivc and iv variables needed for AES.
    aes.get_IV(iv);     //This function return the IV @param out byte pointer that gets the IV. @return none but the IV is writed to the out pointer.

    cout << "IV: " ;
    for(int i =0; i<N_BLOCK;i++){
        printf("%d",iv[i]);
    }
    //aes.do_aes_encrypt(plain,sizeOfPayload,cipher,key,bits,iv); // encrypt (AES) data in plain[] and write it into cipher[]
    aes.do_aes_decrypt(cipher,sizeOfPayload,check,key,bits); // encrypt (AES) data in plain[] and write it into cipher[]

    return cipher;
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
                printf("Received Number of Bytes: %x\n", labb_rfm95.readRegister(RH_RF95_REG_13_RX_NB_BYTES));
                printf("FiFo Current Rx Addr: %x\n", labb_rfm95.readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
                printf("FiFo Addr Ptr: %x\n", labb_rfm95.readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));

            }
            labb_rfm95.handleInterrupt();
            labb_rfm95.rxReceivedLoRaPackage(byteBuffer);

            ///copy the rec

            ///decrypt the received Bytes
            decryDataArrPtr = decrypData(byteBuffer, (int) labb_rfm95.getBufLen());

            //print buffer
            printf("Buffer:\n");

            //charArrPtr = labb_rfm95.convertByteBufToCharBuf(byteBuffer, bufLen);
            charArrPtr = labb_rfm95.convertByteBufToCharBuf(decryDataArrPtr, (int) labb_rfm95.getBufLen());
            //labb_rfm95.printCharBuffer(charArrPtr, bufLen);
            labb_rfm95.printCharBuffer(charArrPtr, (int) labb_rfm95.getBufLen());

            ///fill the charBuffer to all 0
            labb_rfm95.clearCharBuffer(charArrPtr);
        }

    }

    return 0;
}


