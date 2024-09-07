// speccy_comm_mcu.c
// Use a PIC18F16Q41 as the communication processor for the 
// differential-spectrometer data acquisition board.
//
// PJ, 2024-09-07: Implement the basic command interpreter.
//
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = OFF

// CONFIG8
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG9
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include <stdio.h>
#include <string.h>

#define VERSION_STR "v0.1 PIC18F16Q41 SPECTROMETER COMMS-MCU 2024-09-07"

#define GREENLED (LATAbits.LATA2)
#define RESTARTn (LATBbits.LATB7)
#define CS0n (LATCbits.LATC5)
#define CS1n (LATCbits.LATC4)
#define CS2n (LATCbits.LATC3)
#define CS3n (LATCbits.LATC6)
#define CS4n (LATCbits.LATC7)

void init_pins()
{
    // RA2 as digital-output for GREENLED.
    TRISAbits.TRISA2 = 0;
    GREENLED = 0;
    //
    // RB7 as digital-output for restart of DAQ_MCU
    ODCONBbits.ODCB7 = 1;
    RESTARTn = 1;
    TRISBbits.TRISB7 = 0;
    ANSELBbits.ANSELB7 = 0;
    //
    // SPI slave-select pins RC3 through RC7; deselect all.
    ANSELCbits.ANSELC3 = 0; TRISCbits.TRISC3 = 0; LATCbits.LATC3 = 1;
    ANSELCbits.ANSELC4 = 0; TRISCbits.TRISC4 = 0; LATCbits.LATC4 = 1;
    ANSELCbits.ANSELC5 = 0; TRISCbits.TRISC5 = 0; LATCbits.LATC5 = 1;
    ANSELCbits.ANSELC6 = 0; TRISCbits.TRISC6 = 0; LATCbits.LATC6 = 1;
    ANSELCbits.ANSELC7 = 0; TRISCbits.TRISC7 = 0; LATCbits.LATC7 = 1;
}

void select_avr(uint8_t i)
{
    switch (i) {
        case 0: CS0n = 0; break;
        case 1: CS1n = 0; break;
        case 2: CS2n = 0; break;
        case 3: CS3n = 0; break;
        case 4: CS4n = 0; break;
    }
}

void deselect_avr(uint8_t i)
{
    switch (i) {
        case 0: CS0n = 1; break;
        case 1: CS1n = 1; break;
        case 2: CS2n = 1; break;
        case 3: CS3n = 1; break;
        case 4: CS4n = 1; break;
    }
}

void spi1_init()
{
    // SCK1 -> RB6
    // SDI1 <- RB4
    // SDO1 -> RB5
    ANSELBbits.ANSELB6 = 0; TRISBbits.TRISB6 = 0; LATBbits.LATB6 = 0; // SCK
    ANSELBbits.ANSELB4 = 0; TRISBbits.TRISB4 = 1; WPUBbits.WPUB4 = 1; // SDI
    ANSELBbits.ANSELB5 = 0; TRISBbits.TRISB5 = 0; LATBbits.LATB5 = 0; // SDO
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    SPI1SCKPPS = 0b001110; // RB6
    RB6PPS = 0x1b; // SPI1 SCK
    SPI1SDIPPS = 0b001100; // RB4
    RB5PPS = 0x1c; // SPI1 SDO
    // Don't assign an output pin for SPI1SS.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    // Set up SPI1 as master in mode 0.
    SPI1CON0bits.EN = 0; // Turn off to configure
    SPI1CON0bits.SPI1BMODE = 0; // total bit count mode
    SPI1CON0bits.MST = 1; // Master
    SPI1CON1bits.CKE = 1; // data out on clock active to idle
    SPI1CON1bits.SMP = 0; // input is sampled ad middle of data output time 
    SPI1CON2bits.TXR = 1;
    SPI1CON2bits.RXR = 1;
    SPI1CLKbits.CLKSEL = 0b00000; // Fosc system clock
    SPI1BAUD = 32; // 64MHz/32 = 2MHz SPI baud rate
    SPI1STATUSbits.CLRBF = 1; // Clear buffers
    SPI1CON0bits.EN = 1; // Turn on peripheral.
}

void spi1_close()
{
    SPI1CON0bits.EN = 0;
}

uint8_t spi1_exch_byte(uint8_t outgoing_byte)
{
    while (SPI1CON2bits.BUSY) { /* just wait */ }
    SPI1TCNT = 1;
    SPI1TXB = outgoing_byte;
    while (!SPI1INTFbits.SRMTIF) { /* wait for shift register to empty */ }
    uint8_t incoming_byte = SPI1RXB;
    SPI1INTFbits.SRMTIF = 0;
    return incoming_byte;
}
void spi1_exch_buffers(uint8_t* inbuf, uint8_t* outbuf, uint8_t n)
{
    for (uint8_t i = 0; i < n; ++i) {
        *(inbuf+i) = spi1_exch_byte(*(outbuf+i));
    }
}

// For incoming UART comms
#define NBUFA 80
char bufA[NBUFA];
// For outgoing UART comms
#define NBUFB 268
char bufB[NBUFB];
// For incoming SPI data
#define NBUFC 64
uint8_t bufC[NBUFC];
// For outgoing SPI data
#define NBUFD 64
uint8_t bufD[NBUFD];

void interpret_command(char* cmdStr)
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    char number_str[10];
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "v %s\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached AVR MCU.
            RESTARTn = 0;
            __delay_ms(1);
            RESTARTn = 1;
            // Wait until we are reasonably sure that the AVR has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            nchar = snprintf(bufB, NBUFB, "R DAQ_MCU restarted\n");
            uart1_putstr(bufB);
            break;
        case 'L':
            // Turn LED on or off.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                GREENLED = i;
                nchar = snprintf(bufB, NBUFB, "L %d\n", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "L error: no value\n");
            }
            uart1_putstr(bufB);
            break;
        case 'X':
            // Exchange bytes with one of the AVR MCUs.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some text, use it for chip selection.
                uint8_t csi = (uint8_t) atoi(token_ptr);
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    // Found some text, assume it is the first byte value.
                    uint8_t j = 0;
                    while (token_ptr) {
                        i = (uint8_t) atoi(token_ptr);
                        *(bufD + j) = i;
                        ++j;
                        token_ptr = strtok(NULL, sep_tok);
                    }
                    nchar = printf("DEBUG Found %d bytes\n", j);
                    select_avr(csi);
                    spi1_exch_buffers(bufC, bufD, j);
                    deselect_avr(csi);
                    nchar = printf("DEBUG after SPI exchange\n");
                    nchar = snprintf(bufB, NBUFB, "X");
                    for (i=0; i < j; ++i) {
                        nchar = snprintf(number_str, 10, " %02x", bufC[i]);
                        if ((int)strlen(bufB)+nchar > NBUFB-2) break;
                        strncat(bufB, number_str, 10);
                    }
                    nchar = snprintf(number_str, 10, "\n");
                    strncat(bufB, number_str, 10);
                } else {
                    nchar = snprintf(bufB, NBUFB, "X Exchange bytes error: no bytes given.\n");
                }
            } else {
                nchar = snprintf(bufB, NBUFB, "X Exchange bytes error: no chip selected.\n");
            }
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "%c error: Unknown command.\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
} // end interpret_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(230400);
    spi1_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        GREENLED = 1;
        __delay_ms(250);
        GREENLED = 0;
        __delay_ms(250);
    }
    // Wait until we are reasonably sure that the AVR has restarted.
    __delay_ms(100);
    // We wait for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // NL (Ctrl-J) signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            // Note that the incoming string may be of zero length,
            // with the null character in the first place.
            // If that is the case, do nothing with it.
            if (*bufA) {
                interpret_command(bufA);
            }
        }
    }
    uart1_flush_rx();
    spi1_close();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
