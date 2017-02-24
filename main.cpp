#include <iostream>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "Labp_RFM95.h"


using namespace std;

string getOutputfileName(int argcount, char **argvector){

    string filename = "Measurements1.txt";

    if (argcount < 5) {
        return NULL;
    }

    for (int i = 1; i < argcount; ++i) {
        if (string(argvector[i]) == "-o") {
            cout<< argvector[i]<<endl;
            if( i +1 < argcount){ // Make sure we aren't at the end of argv!
                filename =  argvector[(i+1)]; // Increment 'i' so we don't get the argument as the next argv[i].
                return filename;
            }
        }
    }

    return "Measurements4.txt";
}


int main(int argc, char* argv[]) {
    ///check argu for outputfile Name
    string outFileName = getOutputfileName(argc, argv);
    /// print the name
    cout << outFileName << endl;

    ///create instance of the Labb_RFM95 class
    Labp_RFM95 labb_rfm95(6,7,0, outFileName);

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


