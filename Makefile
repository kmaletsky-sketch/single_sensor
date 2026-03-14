TARGET=main

OBJS=$(TARGET).o
ELF=$(TARGET).elf
HEX=$(TARGET).hex

# The frequency at which the Tiny402 will be running. This doesn't change the frequency
#  but rather makes the delay() routine work accurately
F_CPU=20000000L


MCU=attiny402
CFLAGS=-mmcu=attiny402 -B ../Atmel.ATtiny_DFP.1.6.326/gcc/dev/attiny402/ -O3
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

$(HEX): $(ELF)
	$(OBJTOOL) -O ihex -R .eeprom $< $@
	
flash:  main.hex
	pymcuprog write -v debug -d attiny402 -t uart -u /dev/cu.wchusbserial8310 -c 115k --erase --verify -f $(TARGET).hex

clean:
	rm -rf $(OBJS) $(ELF) $(HEX)