TARGET = bootloader
LSCRIPT = ../Linker_Scripts/bootloader.ld

CC = arm-none-eabi-gcc
OS = arm-none-eabi-size
CFLAGS = -g -Wall -mthumb -mcpu=cortex-m3 -O0
ASFLAGS = -Wall -mthumb -mcpu=cortex-m3 -O0
LFLAGS = -Wall -mthumb -mcpu=cortex-m3 -nostdlib -lgcc -T$(LSCRIPT)

OBJS = bootloader.o startup_stm32f103xb.o
HEADERS = ../Device_Headers/stm32f1xx.h ../Linker_Scripts/bootloader.ld ../Linker_Scripts/memory_map.ld

all: $(TARGET).elf

%.o: %.s
	$(CC) -x assembler-with-cpp -c $(ASFLAGS) $< -o $@

%.o: %.c $(HEADERS)
	$(CC) -c $(CFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@
	$(OS) $@

clean:
	@rm -f $(OBJS) $(TARGET).elf

.PHONY: all clean
