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
    Labb_RFM95 labb_rfm95(6,7,0);
    cout << "Hello, World!" << endl;
    labb_rfm95.printAllRegisters();
    return 0;
}


