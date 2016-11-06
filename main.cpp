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

#include "RFM95registers.h"
#include "Labb_RFM95.h"


using namespace std;



int main() {

    //WiringPi needs to be setup
    //wiringPiSetup();

    Labb_RFM95 labb_rfm95(6,7,0);

    //wait until RF95 is resetted
    while(!labb_rfm95.resetRFM95());
    cout << "Starts the RFM95!" << endl;
    // prints all registers of the for debugging
    labb_rfm95.printAllRegisters();


    ///initsRFM95 with the standart parameters
    //labb_rfm95.defaultLoRaSetup();

    uint32_t frequncy = 868100000;
    uint8_t spreadingFac = 7;
    uint8_t codingRade = 6;
    labb_rfm95.loraSetup(frequncy, spreadingFac, codingRade);

/*
    labb_rfm95.setFrequency(frequncy);
    labb_rfm95.setSpredingFactor(spreadingFac);
    labb_rfm95.setCodingRate(codingRade);*/

    char charBuffer[RH_RF95_MAX_PAYLOAD_LEN];
    uint8_t byteBuffer[RH_RF95_MAX_PAYLOAD_LEN];

    int bufLen = RH_RF95_MAX_PAYLOAD_LEN;


    cout <<"******************"<< endl;
    labb_rfm95.printAllRegisters();
    printf("RH_RF95_REG_1D_MODEM_CONFIG1 : %X \n ", labb_rfm95.readRegister(RH_RF95_REG_1D_MODEM_CONFIG1));

    while(1){

        //busy waiting, when Pin 0 goes HIGH it contiuous
        //while(digitalRead(dio0));
        //check if interrupt flag has been set
        //RFM95 Modul sets DIO0 pin (check pinlayout on the breakout board [Adafruit RFM9x -> D]to high when message arrives

        if(digitalRead(7) == true)//labb_rfm95.getIRQpin()
        {

            cout <<"hello" <<endl;
            labb_rfm95.handleInterrupt();


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
            //labb_rfm95.handleInterrupt();
            labb_rfm95.rxReceivedLoRaPackage(byteBuffer);

            //print buffer
            printf("Buffer: \n ");
            static int i;

            for(i=0; i < bufLen;i++){

                charBuffer[i] = (char) byteBuffer[i];
                printf("%c ", charBuffer[i]);
            }
            //fill the charBuffer to all 0
            labb_rfm95.clearCharBuffer(charBuffer);
        }

    }


    return 0;
}


