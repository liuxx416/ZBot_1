PROGRAM = envirovision

SOURCES = envirovision.cpp
SOURCES+= envirobot.cpp
SOURCES+= xiApiPlusOcv.cpp
SOURCES+= eye.cpp
SOURCES+= cpg.cpp
SOURCES+= motor.cpp
SOURCES+= laterPT.cpp
SOURCES+= behaviourDeterminator.cpp
SOURCES+= remregs_serial.cpp
SOURCES+= AD.cpp
SOURCES+= ina219.cpp

OBJECTS = $(SOURCES:.cpp=.o)

CURRENT_DIR = $(shell pwd)

INCLUDE_DIRS = -I$(CURRENT_DIR)/cscorelibs_envirovision/libraries/ntcore/ntcore/src/main/native/include/
INCLUDE_DIRS+= -I$(CURRENT_DIR)/cscorelibs_envirovision/libraries/wpiutil/wpiutil/src/main/native/include/
INCLUDE_DIRS+= -I$(CURRENT_DIR)/cscorelibs_envirovision/libraries/cscore/cscore/src/main/native/include/
INCLUDE_DIRS+= -I$(CURRENT_DIR)/cscorelibs_envirovision/libraries/motor/include/dynamixel_sdk/

LIB_DIRS = -L$(CURRENT_DIR)/cscorelibs_envirovision/build/libraries/ntcore/
LIB_DIRS+= -L$(CURRENT_DIR)/cscorelibs_envirovision/build/libraries/wpiutil/
LIB_DIRS+= -L$(CURRENT_DIR)/cscorelibs_envirovision/build/libraries/cscore/
LIB_DIRS+= -L$(CURRENT_DIR)/cscorelibs_envirovision/build/linux64/
LIB_DIRS+= -ldxl_sbc_c
LIB_DIRS+= -lrt

CC = g++ -std=gnu++11

## debug mode
#COMMON_FLAGS = -g3 -DLINUX -D_GNU_SOURCE -Wall -fmessage-length=0
## optimized mode
COMMON_FLAGS = -O2 -DLINUX -D_GNU_SOURCE -Wall -fmessage-length=0  -lbcm2835 -lm

LNKFLAGS = $(COMMON_FLAGS) #-Wl,-rpath,$(DIR_THOR)/lib

## export PATH:=$(shell echo $$PATH:.)

all: $(PROGRAM)

$(PROGRAM): xiApiPlusOcv.o remregs_serial.o eye.o cpg.o motor.o behaviourDeterminator.o laterPT.o envirobot.o envirovision.o AD.o ina219.o
	$(CC) $(OBJECTS) $(LNKFLAGS) -o $(PROGRAM) -lcscore -lwpiutil -lntcore -lpthread `pkg-config opencv --cflags --libs` -lm3api -lopencv_core -lopencv_imgproc $(LIB_DIRS)

xiApiPlusOcv.o: xiApiPlusOcv.cpp
	$(CC) -c xiApiPlusOcv.cpp $(COMMON_FLAGS) -I . -I /usr/local/include/

remregs_serial.o: remregs_serial.cpp
	$(CC) -c remregs_serial.cpp $(COMMON_FLAGS)

eye.o: eye.cpp
	$(CC) -c eye.cpp $(COMMON_FLAGS) 

cpg.o: cpg.cpp
	$(CC) -c cpg.cpp $(COMMON_FLAGS)

motor.o: motor.cpp
	$(CC) -c motor.cpp $(COMMON_FLAGS) . $(INCLUDE_DIRS)

behaviourDeterminator.o: behaviourDeterminator.cpp
	$(CC) -c behaviourDeterminator.cpp $(COMMON_FLAGS)

laterPT.o: laterPT.cpp
	$(CC) -c laterPT.cpp $(COMMON_FLAGS)

envirobot.o: envirobot.cpp
	$(CC) -c envirobot.cpp $(COMMON_FLAGS) . $(INCLUDE_DIRS)

envirovision.o: envirovision.cpp
	$(CC) -c envirovision.cpp $(COMMON_FLAGS) . $(INCLUDE_DIRS)

AD.o: AD.cpp
	$(CC) -c AD.cpp $(COMMON_FLAGS)

ina219.o: ina219.cpp
	$(CC) -c ina219.cpp $(COMMON_FLAGS) . $(INCLUDE_DIRS)

clean:
	rm -f *.o *~ $(PROGRAM) $(OBJECTS)
