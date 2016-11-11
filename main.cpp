#include <iostream>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "Labb_RFM95.h"


using namespace std;


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


    while(1){
        /// RFM95 Modul sets DIO0 pin (check pinlayout on the breakout board [Adafruit RFM9x -> D]to high when message arrives
        ///check if interrupt flag has been set

        labb_rfm95.mainLoRaHandler();
    }

    return 0;
}


