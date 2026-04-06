TARGET=single

OBJS=$(TARGET).o
ELF=$(TARGET).elf
HEX=$(TARGET).hex

# The frequency at which the Tiny402 will be running. This doesn't change the frequency
#  but rather makes the delay() routine work accurately
F_CPU=20000000L


# -O3 causes the code to increase in size drastically, though I haven't analyzed
#  the reason for this to be
MCU=attiny402
CFLAGS=-mmcu=attiny402 -B ../Atmel.ATtiny_DFP.1.6.326/gcc/dev/attiny402/ -O2 -Wall
CFLAGS+=-I ../Atmel.ATtiny_DFP.1.6.326/include/ -DF_CPU=$(F_CPU)
LDFLAGS=-mmcu=attiny402 -B ../Atmel.ATtiny_DFP.1.6.326/gcc/dev/attiny402/

CC=avr-gcc
LD=avr-gcc
OBJTOOL=avr-objcopy

all: $(HEX)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ 

$(ELF):	$(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)
	avr-size $(ELF)

$(HEX): $(ELF)
	$(OBJTOOL) -O ihex -R .eeprom $< $@
	
flash:  $(HEX)
	pymcuprog write -d attiny402 -t uart -u /dev/cu.wchusbserial8310 -c 56k --erase --verify -f $(HEX)

clean:
	rm -rf $(OBJS) $(ELF) $(HEX)


# pymcuprog is developed and officially maintained by Microchip
#   add   -v debug     to enable debug mode
# pymcuprog write -v debug -d attiny402 -t uart -u /dev/cu.wchusbserial8310 -c 56k --erase --verify -f $(HEX)

# This also works
#	avrdude -v -C /opt/local/etc/avrdude.conf -c serialUPDI -P /dev/cu.wchusbserial8310 -p t402 -U flash:w:$(HEX)