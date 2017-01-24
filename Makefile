CC = g++
OBJS = main.cpp Labp_RFM95.cpp AES.cpp
CFLAGS  = -g -Wall -std=gnu++11
LIBS    = -g -lwiringPi

output: main.cpp
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

debug:
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

comprun:
	rm main && $(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main
	sudo ./main -f 868100000 -sf 12 -cr 5 -bw 500
run:
	sudo ./main -f 868100000 -sf 12 -cr 5 -bw 500

clean:
	rm main