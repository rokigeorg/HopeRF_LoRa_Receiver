CC = g++
OBJS = main.cpp Labb_RFM95.cpp AES.cpp
CFLAGS  = -g -Wall -std=gnu++11
LIBS    = -g -lwiringPi

output: main.cpp
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

debug:
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main

comprun:
	rm main && $(CC) $(CFLAGS) $(LIBS) $(OBJS) -o main
	sudo ./main -f 868100000 -sf 7 -cr 5
run:
	sudo ./main -f 868100000 -sf 7 -cr 5

clean:
	rm main