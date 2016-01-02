PROG := HBridgeDriver

include ../Makefile.common

CORE_OBJS= \
	wiring.o wiring_analog.o wiring_shift.o wiring_pulse.o wiring_digital.o \
	abi.o hooks.o new.o Stream.o HID.o Print.o CDC.o IPAddress.o USBCore.o Tone.o \
	WInterrupts.o WMath.o WString.o \
	HardwareSerial.o HardwareSerial0.o HardwareSerial1.o HardwareSerial2.o HardwareSerial3.o \
	main.o

CORE_LIB=core.a

OBJS=$(PROG).o
ELF=$(PROG).elf
EEP=$(PROG).eep
HEX=$(PROG).hex

.PHONY: all clean build upload

all: build

build: $(ELF) $(EEP) $(HEX)

$(ELF): $(OBJS) $(CORE_LIB)
	$(CC) -Wall -Wextra -Os -Wl,--gc-sections -mmcu=atmega328p -o $(ELF) $(OBJS) $(CORE_LIB) -lm 

$(EEP): $(ELF)
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(ELF) $(EEP)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(ELF) $(HEX)

$(CORE_LIB): $(CORE_OBJS)
	$(AR) rcs $@ $^
	
clean:
	-rm $(OBJS)
	-rm $(CORE_OBJS) 
	-rm $(CORE_LIB)
	-rm $(ELF)
	-rm $(EEP)
	-rm $(HEX)
	-rm *.d

upload: $(EEP) $(HEX)
	$(AVRDUDE) -C$(ARDUINO_HOME)/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -carduino -P$(PORT) -b$(BAUD) -D -Uflash:w:$(HEX):i 

$(PROG).o: $(PROG).cpp

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $(DEFINES) $(INCLUDES) -o $@ $<

%.o: $(ARDUINO_SRC)/%.cpp
	$(CPP) $(CPPFLAGS) $(DEFINES) $(INCLUDES) -o $@ $<

%.o: $(ARDUINO_SRC)/%.c
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -o $@ $<

