TARGET = blink
LSCRIPT = ../Linker_Scripts/app.ld

CC = arm-none-eabi-gcc
OC = arm-none-eabi-objcopy
OS = arm-none-eabi-size
CFLAGS = -g -Wall -mthumb -mcpu=cortex-m3 -O0
ASFLAGS = -Wall -mthumb -mcpu=cortex-m3 -O0
LFLAGS = -Wall -mthumb -mcpu=cortex-m3 -nostdlib -lgcc -T$(LSCRIPT)

OBJS = blink.o startup_stm32f103xb.o
HEADERS = ../Device_Headers/stm32f1xx.h ../Linker_Scripts/app.ld ../Linker_Scripts/memory_map.ld

all: $(TARGET).bin

%.o: %.s
	$(CC) -x assembler-with-cpp -c $(ASFLAGS) $< -o $@

%.o: %.c $(HEADERS)
	$(CC) -c $(CFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

clean:
	@rm -f $(OBJS) $(TARGET).elf $(TARGET).bin

.PHONY: all clean
