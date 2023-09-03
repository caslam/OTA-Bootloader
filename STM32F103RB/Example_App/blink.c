#include <stdint.h>
#include "../Device_Headers/stm32f1xx.h"

// Prescalar for the AHB clock fed to the Cortex Timer.
#define SYSTICK_CLOCK_SOURCE_SCALAR 8

// Scalar for the fraction of a second at which SysTick exception will trigger.
#define SYSTICK_COUNTER_SCALAR 10

// Use the default 8 MHz HSI source.
uint32_t SystemCoreClock = 8000000;

// SysTick exception counter.
volatile int st_counter = 0;

// Takes an int representing time in 0.1 seconds to delay. 
void Delay(int st_counter_val);

// Reloads the SysTick counter to trigger an exception every 100ms.
void SysTickStart();

// Disables the SysTick exception and resets "st_counter."
void SysTickStop();

// This program takes input from B1 to toggle LED2 on/off.
int main(void) {
  Delay(20);
  // Enable GPIO ports A and C.
  RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN);

  // MODE5 01: Output mode, max speed 10 MHz
  // CNF5  00: General purpose output push-pull
  GPIOA->CRL |= GPIO_CRL_MODE5_0;   
  GPIOA->CRL &= ~GPIO_CRL_CNF5_0;

  // MODE13 00: Input mode
  // CNF13  10: Input with pull-up / pull-down
  // ODR13   1:            pull-up
  GPIOC->CRH |= GPIO_CRH_CNF13_1;
  GPIOC->CRH &= ~GPIO_CRH_CNF13_0;
  GPIOC->ODR |= GPIO_ODR_ODR13;
  
  uint8_t button_pressed = 0;
  while (1) {
    uint32_t idr_b13 = GPIOC->IDR & GPIO_IDR_IDR13;
    if (!idr_b13) {
      // The button is pressed; if it was not already pressed, change the LED
      // state.
      if (!button_pressed) {
        GPIOA->ODR ^= GPIO_ODR_ODR5;
      }
      button_pressed = 1;
    }
    else {
      button_pressed = 0;
    }
  }
}

void SysTick_Handler() {
    st_counter++;
}

void Delay(int st_counter_val) {
    SysTickStart();
    while (st_counter < st_counter_val) { }
    SysTickStop();
}

void SysTickStart() {
    // Trigger SysTick_Handler every 100 ms.
    SysTick->LOAD |= ((SystemCoreClock / SYSTICK_CLOCK_SOURCE_SCALAR)
                       / SYSTICK_COUNTER_SCALAR) - 1;
    // Clear the current value.
    SysTick->VAL |= 0;
    // Enable exception trigger and start the counter.
    SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

void SysTickStop() {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    st_counter = 0;
}
