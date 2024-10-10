#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void GPIOPortF_Handler(void) {
    // Check if interrupt occurred on PF4 (Switch 1)
    if (GPIO_PORTF_RIS_R & (1 << 4)) {
        GPIO_PORTF_ICR_R |= (1 << 4);   // Clear the interrupt flag for PF4
        UART1_CTL_R &= ~(0x01);         // Disable UART
        while (UART1_FR_R & (0x08)){};  // Wait until UART is not busy (TXFE)
        UART1_LCRH_R &= ~(0x10);        // Flush the FIFO
        UART1_CTL_R |= (0x80);          // Enable Tx
        UART1_DR_R = (0xF0);            // Load the data into the data register
        UART1_CTL_R |= (0x01);          // Enable the UART
    }

    // Check if interrupt occurred on PF0 (Switch 2)
    if (GPIO_PORTF_RIS_R & (1 << 0)) {
        GPIO_PORTF_ICR_R |= (1 << 0);   // Clear the interrupt flag for PF0
        UART1_CTL_R &= ~(0x01);         // Disable UART
        while (UART1_FR_R & (0x08)){};  // Wait until UART is not busy (TXFE)
        UART1_LCRH_R &= ~(0x10);        // Flush the FIFO
        UART1_CTL_R |= (0x80);          // Enable Tx
        UART1_DR_R = (0xAA);            // Load the data into the data register
        UART1_CTL_R |= (0x01);          // Enable the UART
    }
}

int main(void) {
    // Enable clock for Port F
    SYSCTL_RCGCGPIO_R |= (1 << 5);
    while ((SYSCTL_PRGPIO_R & (1 << 5)) == 0);

    // Unlock PF0 and PF4 (Switches)
    GPIO_PORTF_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port F
    GPIO_PORTF_CR_R |= 0x11;         // Allow changes to PF0 and PF4

    // Configure PF0 and PF4 (Switches) as input and PF1 (LED) as output
    GPIO_PORTF_DIR_R &= ~(0x11);     // Set PF0 and PF4 as input (switches)
    GPIO_PORTF_DIR_R |= (0x02);      // Set PF1 as output (LED)
    GPIO_PORTF_DEN_R |= (0x13);      // Enable digital functionality for PF0, PF4, and PF1
    GPIO_PORTF_PUR_R |= (0x11);      // Enable pull-up resistors on PF0 and PF4

    // Enable clock for UART1 and Port B
    SYSCTL_RCGCUART_R |= (1 << 1);   // Enable UART1 clock
    SYSCTL_RCGCGPIO_R |= (1 << 1);   // Enable GPIO Port B clock
    while ((SYSCTL_PRGPIO_R & (1 << 1)) == 0);

    // Configure PB0 and PB1 for UART1
    GPIO_PORTB_AFSEL_R |= (1 << 0) | (1 << 1);  // Enable alternate function on PB0 and PB1
    GPIO_PORTB_PCTL_R |= (1 << 0) | (1 << 4);   // Set UART functionality on PB0 (U1Rx) and PB1 (U1Tx)
    GPIO_PORTB_DEN_R |= (1 << 0) | (1 << 1);    // Enable digital functionality on PB0 and PB1

    // Configure UART1
    UART1_CTL_R &= ~(0x01);            // Disable UART1
    UART1_IBRD_R = 104;                // Integer portion of BRD (for 9600 baud rate at 16 MHz clock)
    UART1_FBRD_R = 11;                 // Fractional portion of BRD
    UART1_LCRH_R = (0x3 << 5);         // 8-bit, no parity, 1 stop bit
    UART1_CC_R = 0x0;                  // Use system clock for UART
    UART1_CTL_R |= (0x01 | 0x200);     // Enable UART1 and Tx

    // Configure interrupt for PF0 and PF4
    GPIO_PORTF_IM_R &= ~(0x11);        // Disable interrupts for PF0 and PF4
    GPIO_PORTF_IS_R &= ~(0x11);        // Make PF0 and PF4 edge-sensitive
    GPIO_PORTF_IBE_R &= ~(0x11);       // Not both edges
    GPIO_PORTF_IEV_R &= ~(0x11);       // Falling edge triggers
    GPIO_PORTF_ICR_R |= 0x11;          // Clear any prior interrupts
    GPIO_PORTF_IM_R |= 0x11;           // Enable interrupts on PF0 and PF4

    // Enable GPIO Port F interrupt in NVIC (IRQ30 for Port F)
    NVIC_EN0_R |= (1 << 30);

    // Enable global interrupts
    __asm("     CPSIE I");

    while (1) {
        // Main loop
    }
}
