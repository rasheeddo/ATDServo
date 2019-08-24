CC=g++

CFLAGS= -c -Wall

OUTPUT=testServo

all: $(OUTPUT)

$(OUTPUT): main.o ATServoClass.o
	$(CC) main.o ATServoClass.o -o $(OUTPUT)

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

ATServoClass.o: ATServoClass.cpp
	$(CC) $(CFLAGS) ATServoClass.cpp 

clean:
	rm -rf *o $(OUTPUT)