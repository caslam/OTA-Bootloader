#include <stdint.h>
#include "../Device_Headers/stm32f1xx.h"

// Prescalar for the AHB clock fed to the Cortex Timer.
#define SYSTICK_CLOCK_SOURCE_SCALAR 8

// Scalar for the fraction of a second at which SysTick exception will trigger.
#define SYSTICK_COUNTER_SCALAR 10

// Baud rate register values; results in baud rate of 9600 bits/s with 0.03%
// error.
#define DIV_MANTISSA 52
#define DIV_FRACTION 1

// Number of bytes in one page of flash.
#define PAGE_SIZE 1024

// Value in flash after an erase operation.
#define FLASH_RESET_VALUE 0xFFU

// Bytes in a half word.
#define HALF_WORD 2

// Size of the UART read/write buffer.
#define BUF_SIZE 1024

// Byte sent to request data from the ESP32.
#define TX_BYTE 0xEE

// Maximum size of the application binary (128kB of flash - 16kB set aside 
// for the bootloader).
#define MAX_FILE_SIZE 114688

// Error codes.
#define OK 0
#define ERROR -1

////////////////////////////////////////////////////////////////////////////////
// Global variables:

// Variables holding start addresses and sizes of bootloader/application memory
// regions, defined in the linker script.
extern char bootrom_start;
extern char bootrom_size;
extern char approm_start;
extern char approm_size;

// Use the default 8 MHz HSI source.
uint32_t SystemCoreClock = 8000000;

// SysTick exception counter.
volatile int st_counter = 0;

////////////////////////////////////////////////////////////////////////////////
// Helper function prototypes:

// Takes two uint32_ts representing the program counter and stack pointer of 
// the application. Loads the app's stack pointer and jumps to its reset
// handler.
void StartApp(uint32_t pc, uint32_t sp);

// Sets PA_5 as output push-pull for the user LED (LD2) and PC_0 as an
// input with pull-down for the signal to load a new program.
void ConfigGPIO();

// Enables USART3. Sets PB_10 (USART3_Tx) as an alternate function push-pull
// output. Baud rate is set to 9600 bits/s; 8 data bits, 1 stop bit, no parity. 
void ConfigUART();

// Reloads the SysTick counter to trigger an exception every 100ms.
void SysTickStart();

// Disables the SysTick exception and resets "st_counter."
void SysTickStop();

// Unlocks/re-locks the flash program and erase controller. UnlockFlash()
// returns "ERROR" if the flash controller is still locked, or "OK" otherwise.
int UnlockFlash();
void LockFlash();

// Erases the previous app, receives a new program from the ESP32 over USART3,
// and places it in the app ROM section. Returns "ERROR" if an error occurred
// during the process, or "OK" otherwise.
int LoadNewProgram();

// Erases the entire app ROM space one page at a time. Returns "ERROR" if a
// page was not fully erased, or "OK" otherwise.
int EraseAppROM();

// Sends "TX_BYTE" over USART3.
void SendTXByte();

// Requests and returns a 4-byte big-endian uint from the ESP32 representing
// the size of the app binary in bytes.
uint32_t GetProgramSize(uint8_t *buf);

// Requests up to "BUF_SIZE" number of bytes from the ESP32 and writes them to
// a buffer. Returns the number of bytes read, or "ERROR" if an overrun error
// occurred.
int GetProgram(uint8_t *buf, int bytes_to_read);

// Writes "bytes_to_write" number of bytes from the buffer to the given start
// address in flash, where "start_addr" is the lowest address that will be
// written to. Writes to flash are required to be 16 bits, so "bytes_to_write"
// must be evenly divisible by 2. Returns the number of bytes written, or
// "ERROR" if the write was unsuccessful. 
int WriteProgram(uint8_t*buf, int bytes_to_write, uint32_t start_addr);

// Resets peripheral registers used by the bootloader.
void ResetRegisters();

////////////////////////////////////////////////////////////////////////////////
// Bootloader implementation:

// This program will wait for 0.5 seconds on boot before jumping to the current 
// application. If PC_0 is high during the wait period, it receives a new
// program via USART3 and places it in the app ROM section of flash. The
// program is expected to be sent in the form of a raw binary file (no
// debugging info), 32-bit aligned, and no more than 112kB in size.
int main() {
    ConfigGPIO();
    SysTickStart();
    while (st_counter < 5) {
        uint32_t pc_0 = GPIOC->IDR & GPIO_IDR_IDR0;
        if (pc_0) {
            SysTickStop();
            GPIOA->ODR |= GPIO_ODR_ODR5;  // User LED on
            if (LoadNewProgram() == ERROR) {
                while (1) { }
            }
            break;
        }
    }
    SysTickStop();
    GPIOA->ODR &= ~GPIO_ODR_ODR5;  // LED off
    ResetRegisters();
    // Relocate the vector table to the start of the app ROM section.
    SCB->VTOR |= (uint32_t)(&bootrom_size);
    uint32_t *approm = (uint32_t *)(&approm_start);
    // End of stack is the first word of vector table; reset handler is the
    // second.
    StartApp(approm[1], approm[0]);
    while (1) { }
}

void SysTickStart() {
    SysTick->LOAD |= ((SystemCoreClock / SYSTICK_CLOCK_SOURCE_SCALAR) 
                       / SYSTICK_COUNTER_SCALAR) - 1;
    SysTick->VAL |= 0;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTickStop() {
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    st_counter = 0;
}

void SysTick_Handler() {
    st_counter++;
}

void ConfigGPIO() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL |= GPIO_CRL_MODE5_0;   
    GPIOA->CRL &= ~GPIO_CRL_CNF5_0;

    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRL |= GPIO_CRL_CNF0_1;
    GPIOC->CRL &= ~GPIO_CRL_CNF0_0;
}

void ConfigUART() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    GPIOB->CRH &= ~GPIO_CRH_MODE10_1;
    GPIOB->CRH |= GPIO_CRH_MODE10_0;
    GPIOB->CRH |= GPIO_CRH_CNF10_1;
    GPIOB->CRH &= ~GPIO_CRH_CNF10_0;

    USART3->CR1 |= USART_CR1_UE;

    USART3->BRR |= DIV_MANTISSA << USART_BRR_DIV_Mantissa_Pos;
    USART3->BRR |= DIV_FRACTION << USART_BRR_DIV_Fraction_Pos;

    USART3->CR1 |= USART_CR1_RE | USART_CR1_TE;
}

void ResetRegisters() {
    RCC->APB2RSTR |=  (RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST |
                       RCC_APB2RSTR_IOPCRST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST |
                       RCC_APB2RSTR_IOPCRST);
    RCC->APB1RSTR |=  RCC_APB1RSTR_USART3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

int LoadNewProgram() {
    ConfigUART();
    if (UnlockFlash() == ERROR) {
        return ERROR;
    }
    if (EraseAppROM() == ERROR) {
        return ERROR;
    }
    uint8_t buf[BUF_SIZE];
    uint32_t app_size = GetProgramSize(buf);
    // Check program alignment and size requirements.
    if ((app_size % 4 != 0) || (app_size > MAX_FILE_SIZE)) {
        return ERROR;
    }
    uint32_t left_to_read = app_size;
    uint32_t write_addr = (uint32_t)(&approm_start);
    while ((left_to_read > 0) && (write_addr <= ((uint32_t)(&approm_start)
                                                  + app_size - 2))) {
        // Read a maximum of "BUF_SIZE" bytes of the file at a time until EOF
        // is reached.
        int bytes_read = GetProgram(buf, (left_to_read < BUF_SIZE) ? 
                                          left_to_read : BUF_SIZE);
        int bytes_written = WriteProgram(buf, bytes_read, write_addr);
        if (bytes_read != bytes_written) {
            return ERROR;
        }
        left_to_read -= bytes_written;
        write_addr += bytes_written;
    }
    LockFlash();
    return OK;
}

void SendTXByte() {
    while (!(USART3->SR & USART_SR_TXE)) { }
    USART3->DR |= (uint8_t)TX_BYTE;
    while (!(USART3->SR & USART_SR_TC)) { }
}

uint32_t GetProgramSize(uint8_t *buf) {
    SendTXByte();
    for (int i = 0; i < sizeof(uint32_t); i++) {
        // Wait until data is received.
        while (!(USART3->SR & USART_SR_RXNE)) { }
        buf[i] = USART3->DR;
    }
    // Expect the size to be sent in network order.
    uint32_t size = (buf[3] << 0) | (buf[2] << 8) | (buf[1] << 16) | (buf[0] << 24);
    return size;
}

int GetProgram(uint8_t *buf, int bytes_to_read) {
    int bytes_read = 0;
    SendTXByte();
    for (int i = 0; i < bytes_to_read; i++) {
        // Check for USART overrun error (data lost).
        if ((USART3->SR & USART_SR_ORE)) {
            return ERROR;
        }
        while (!(USART3->SR & USART_SR_RXNE)) { }
        buf[i] = USART3->DR;
        bytes_read++;
    }
    return bytes_read;
}

int WriteProgram(uint8_t *buf, int bytes_to_write, uint32_t start_addr) {
    int bytes_written = 0;
    uint32_t write_addr = start_addr;
    while (FLASH->SR & FLASH_SR_BSY) { }
    for (int i = 0; i < bytes_to_write / 2; i++) {
        FLASH->CR |= FLASH_CR_PG;
        // Copy the buffer contents into flash 16 bits at a time.
        *((uint16_t *)write_addr) = ((uint16_t *)buf)[i];
        while (FLASH->SR & FLASH_SR_BSY) { }
        // Verify that the half word was written.
        if (*((uint16_t *)write_addr) != ((uint16_t *)buf)[i]) {
            return ERROR;
        }
        bytes_written += HALF_WORD;
        write_addr += HALF_WORD;
    }
    return bytes_written;
}

int UnlockFlash() {
    FLASH->KEYR |= FLASH_KEY1;
    FLASH->KEYR |= FLASH_KEY2;
    if (FLASH->CR & FLASH_CR_LOCK) {
        return ERROR;
    }
    return OK;
}

void LockFlash() {
    FLASH->CR |= FLASH_CR_LOCK;
}

int EraseAppROM() {
    int pages_to_erase = (uint32_t)(&approm_size) / PAGE_SIZE;
    uint32_t page_address = (uint32_t)(&approm_start);
    for (int i = 0; i < pages_to_erase; i++) {
        page_address = (uint32_t)(&approm_start) + (i * PAGE_SIZE);
        FLASH->CR |= FLASH_CR_PER;
        // Load the start address of the page to be erased. 
        FLASH->AR |= page_address;
        FLASH->CR |= FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY) { }
        // Verify that the page was fully erased.
        for (uint32_t j = page_address; j < page_address + PAGE_SIZE; j++) {
            if (*((uint8_t *)j) != FLASH_RESET_VALUE) {
                return ERROR;
            }
        }
        // Reset the address register.
        FLASH->AR &= 0UL;
    }
    FLASH->CR &= ~FLASH_CR_PER;
    return OK;
}

__attribute__((naked))
void StartApp(uint32_t pc, uint32_t sp)  {
    // Load r1 ("sp") as the stack pointer and branch to r0 ("pc").
    __asm("msr msp, r1\n\
           bx r0"       
         );
}
