#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void UART1_Init(void);
char UART1_ReadChar(void);
void LED_Init(void);
void LED_Control(char receivedData);

int main(void) {
    // Initialize UART1 and LED pins
    UART1_Init();
    LED_Init();

    while (1) {
        received = UART1_ReadChar();  // Read character from UART1
        LED_Control(received);             // Control LED based on received data
    }
}

void UART1_Init(void) {
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
    UART1_CTL_R |= (0x01 | 0x200);     // Enable UART1 and Rx
}

char UART1_ReadChar(void) {
    while ((UART1_FR_R & 0x10) != 0);  // Wait until the Rx buffer is not empty
    return UART1_DR_R & 0xFF;  // Read the received character
}

void LED_Init(void) {
    // Enable clock for Port F
    SYSCTL_RCGCGPIO_R |= (1 << 5);
    while ((SYSCTL_PRGPIO_R & (1 << 5)) == 0);

    // Configure PF1, PF2, and PF3 as output (Red, Blue, Green LEDs)
    GPIO_PORTF_DIR_R |= 0x0E;          // Set PF1, PF2, PF3 as output
    GPIO_PORTF_DEN_R |= 0x0E;          // Enable digital functionality on PF1, PF2, PF3
}

void LED_Control(char receivedData) {
    // Turn off all LEDs initially
    GPIO_PORTF_DATA_R &= ~(0x0E);

    // Control LEDs based on received data
    if (receivedData == 0xAA) {                  // "aa" received as two 'a' characters
        GPIO_PORTF_DATA_R |= (1 << 3);          // Turn on green LED (PF3)
    } else if (receivedData == 0xF0) {          // "f0" received as a byte
        GPIO_PORTF_DATA_R |= (1 << 2);          // Turn on blue LED (PF2)
    } else {
        GPIO_PORTF_DATA_R |= (1 << 1);          // Turn on red LED (PF1) for error
    }
}
