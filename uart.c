// uart.c
// Functions to provide a shim between the C standard library functions
// and UART1 peripheral device on the PIC16F1/PIC18F microcontroller.
// Build with linker set to link in the C99 standard library (default).
// PJ,
// 2023-12-01 PIC18F16Q41 attached to a MAX3082 RS485 transceiver
// 2024-09-05 Return to using full-duplex and RTS/CTS with an FTDI TTL-232-5V

#include <xc.h>
#include "global_defs.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

void uart1_init(long baud)
{
    // Follow recipe given in PIC18F16Q41 data sheet
    // Sections 34.2.1.8 and 34.2.2.1
    // We are going to use hardware flow control CTSn/RTSn.
    unsigned int brg_value;
    //
    // Configure PPS RX1=RC1, TX1=RC0, CTS1=RA5, RTS1=RA4
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    U1RXPPS = 0b010001; // RC1
    U1CTSPPS = 0b000101; // RA5
    RC0PPS = 0x10; // UART1 TX
    RA4PPS = 0x12; // UART1 RTS
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELCbits.ANSELC0 = 0; // TX1 pin
    TRISCbits.TRISC0 = 0; // output
    ANSELAbits.ANSELA4 = 0; // RTS1 pin
    TRISAbits.TRISA4 = 0; // output
    ANSELCbits.ANSELC1 = 0; // Turn on digital input buffer for RX1
    TRISCbits.TRISC1 = 1; // RX1 is an input
    ANSELAbits.ANSELA5 = 0; // Turn on digital input buffer for CTS1
    TRISAbits.TRISA5 = 1; // CTS1 is an input
    //
    U1CON0bits.BRGS = 1;
    brg_value = (unsigned int) (FOSC/baud/4 - 1);
    // For 64MHz, 115200 baud, expect value of 137.
    //              9600 baud                 1665.
    //            230400 baud                   68 for 0.6% error
    U1BRG = brg_value;
    //
    U1CON0bits.MODE = 0b0000; // Use 8N1 asynchronous
    U1CON2bits.FLO = 0b10; // Hardware flow control (RTS/CTS)
    U1CON0bits.RXEN = 1;
    U1CON0bits.TXEN = 1;
    U1CON1bits.ON = 1;
    return;
}

void uart1_putch(char data)
{
    // Wait until shift-register empty, then send data.
    while (!U1ERRIRbits.TXMTIF) { CLRWDT(); }
    U1TXB = data;
    return;
}

void uart1_putstr(char* str)
{
    for (size_t i=0; i < strlen(str); i++) uart1_putch(str[i]); 
    return;
}

void uart1_flush_rx(void)
{
    U1FIFObits.RXBE = 1;
}

char uart1_getch(void)
{
    char c;
    // Block until a character is available in buffer.
    while (U1FIFObits.RXBE) { CLRWDT(); }
    // Get the data that came in.
    c = U1RXB;
    return c;
}

int uart1_getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a new-line character.
// Returns the number of characters collected,
// excluding the terminating null char.
{
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = uart1_getch();
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\n') {
            done = 1;
            buf[i] = '\0';
        }
        if (c == '\b' && i > 0) {
            // Backspace.
            i--;
        }
    }
    return i;
}

void uart1_close(void)
{
    U1CON0bits.RXEN = 1;
    U1CON0bits.TXEN = 1;
    U1CON1bits.ON = 1;
    return;
}

// Functions to connect STDIO to UART1.

void putch(char data)
{
    uart1_putch(data);
    return;
}

int getch(void)
{
    return uart1_getch();
}

int getche(void)
{
    int data = getch();
    putch((char)data); // echo the character
    return data;
}
