CC = g++
OBJS = main.cpp RFM95registers.h Labb_RFM95.cpp
CFLAGS  = -g -Wall -std=gnu++11
LIBS    = -g -lwiringPi

output: main.cpp
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

debug:
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

comprun:
	rm main && $(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main
	sudo ./main

clean:
	rm main