// speccy_comm_mcu.c
// Use a PIC18F16Q41 as the communication processor for the 
// differential-spectrometer data acquisition board.
//
// PJ, 2024-09-07: Implement the basic command interpreter.
//     2025-09-26: Update to PCB specification.
//     2025-10-18: Remove DEBUG messages on SPI transfers.
//     2025-10-25: Add command 'D' to return all analog data in one message.
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

#define VERSION_STR "v0.6 PIC18F16Q41 SPECTROMETER COMMS-MCU 2025-10-25"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID 'D'

#define LED (LATAbits.LATA4)
uint8_t override_led = 0;

#define AVR_RESETn (LATBbits.LATB7)

#define AVR0_CSn (LATCbits.LATC5)
#define AVR1_CSn (LATCbits.LATC4)
#define AVR2_CSn (LATCbits.LATC3)
#define AVR3_CSn (LATCbits.LATC6)
#define AVR4_CSn (LATCbits.LATC7)

void init_pins()
{
    // RA4 as digital-output for LED.
    TRISAbits.TRISA4 = 0;
    LED = 0;
    //
    // RB7 as digital-output for restart of DAQ_MCU
    ODCONBbits.ODCB7 = 1;
    AVR_RESETn = 1;
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
        case 0: AVR0_CSn = 0; break;
        case 1: AVR1_CSn = 0; break;
        case 2: AVR2_CSn = 0; break;
        case 3: AVR3_CSn = 0; break;
        case 4: AVR4_CSn = 0; break;
        default: /* Do nothing */ {}
            
    }
}

void deselect_avr(uint8_t i)
{
    switch (i) {
        case 0: AVR0_CSn = 1; break;
        case 1: AVR1_CSn = 1; break;
        case 2: AVR2_CSn = 1; break;
        case 3: AVR3_CSn = 1; break;
        case 4: AVR4_CSn = 1; break;
        default: /* Do nothing */ {}
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
    SPI1BAUD = 15; // 64MHz/(2*(15+1)) = 2MHz SPI baud rate
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
#define NBUFC 32
uint8_t bufC[NBUFC];
// For outgoing SPI data
#define NBUFD 32
uint8_t bufD[NBUFD];
// We gather the analog data as a sequence of bytes and just keep them
// as they come from the AVR in (big-endian) order without trying to 
// reinterpret them. 
#define N_ANALOG_DATA_BYTES 80
uint8_t analog_data_bytes[N_ANALOG_DATA_BYTES];

int find_char(char* buf, int start, int end, char c)
// Returns the index of the character if found, -1 otherwise.
// start is the index of the first character to check.
// end is the index of the last character to check.
{
    for (int i = start; i <= end; i++) {
        if (buf[i] == '\0') return -1;
        if (buf[i] == c) return i;
    }
    return -1;
}

char* trim_RS485_command(char* buf, int nbytes)
// Returns a pointer to the command text string, within buf.
// The resulting string may be zero-length.
//
// A valid incoming command from the RS485 will be of the form
// "/cXXXXXXXX!"
// where the components are
//    / is the start character
//    ! is the end character
//    c is the MYID character, identifying the receiving node
//    XXXXXXX is the command text
//
// This format is described in the text:
// J.M. Hughes
// Real World Instrumentation
// O'Rielly 2010
// Chapter 11 Instrumentation Data I/O, Unique Protocols.
//
{
    // printf("DEBUG: buf=%s", buf);
    int start = find_char(buf, 0, nbytes-1, '/');
    if (start == -1) {
        // Did not find starting '/'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    int end = find_char(buf, start, nbytes-1, '!');
    if (end == -1) {
        // Did not find terminating '!'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, we have a valid incoming command.
    if (buf[start+1] != MYID) {
        // The incoming message is not for this node, so discard it.
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, the message is for us.
    buf[end] = '\0'; // Trim off the '!' character.
    // On return, omit the MYID character from the front.
    return &buf[start+2];
} // end trim_command()

void interpret_RS485_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    char number_str[10];
    if (!override_led) LED = 1; // To indicate start of interpreter activity.
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            // Echo the version string for the PIC18 firmware.
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached AVR MCU(s).
            AVR_RESETn = 0;
            __delay_ms(1);
            AVR_RESETn = 1;
            // Wait until we are reasonably sure that the AVR has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#\n");
            uart1_putstr(bufB);
            break;
        case 'L':
            // Turn (local) PIC18 LED on or off.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                LED = i;
                override_led = i;
                nchar = snprintf(bufB, NBUFB, "/0L %d#\n", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "/0L error: no value#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'X':
            // Exchange bytes with one of the AVR MCUs.
            // Form of this command:
            // X <csi> <b0> <b1> <b2> <b3> ...
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
                    // nchar = printf("/0X DEBUG csi=%d plus Found %d bytes to send over SPI#\n", csi, j);
                    select_avr(csi);
                    spi1_exch_buffers(bufC, bufD, j);
                    deselect_avr(csi);
                    // nchar = printf("/0X DEBUG after SPI exchange#\n");
                    nchar = snprintf(bufB, NBUFB, "/0X");
                    for (i=0; i < j; ++i) {
                        nchar = snprintf(number_str, 10, " %02x", bufC[i]);
                        if ((int)strlen(bufB)+nchar > NBUFB-2) break;
                        strncat(bufB, number_str, 10);
                    }
                    nchar = snprintf(number_str, 10, "#\n");
                    strncat(bufB, number_str, 10);
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0X Exchange bytes error: no bytes given.#\n");
                }
            } else {
                nchar = snprintf(bufB, NBUFB, "/0X Exchange bytes error: no chip selected.#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'D' :
            // Gather the analog-sample data from each of the AVRs and report all
            // 40 values in a single message.
            //
            // First, tell all AVRs to load their sample data into their outgoing
            // buffer for SPI transfer.
            for (uint8_t csi=0; csi < 5; ++csi) {
                bufD[0] = 80; // single-character command (decimal 80)
                bufD[1] = 0;
                select_avr(csi);
                spi1_exch_buffers(bufC, bufD, 2);
                deselect_avr(csi);
            }
            // Allow some time for the AVRs to copy data into their buffers.
            __delay_ms(2);
            // Second, go get that data from each AVR.
            // We will accumulate 5x16=80 bytes into analog_data_bytes array.
            i = 0;
            for (uint8_t csi=0; csi < 5; ++csi) {
                bufD[0] = 0; // single-character "nil" command
                // Although we are going to exchange 18 bytes,
                // in order to get all 16 significant bytes back
                // from the AVR, we don't care about the content
                // being sent in the remaining part of bufD.
                select_avr(csi);
                spi1_exch_buffers(bufC, bufD, 18);
                deselect_avr(csi);
                // We ignore the first 2 bytes and keep the next 16 bytes.
                for (j=2; j < 18; ++j) {
                    analog_data_bytes[i] = bufC[j];
                    ++i;
                }
            }
            // Finally, write the byte data into a hex string that gets wrapped
            // into RS485 message format and returned.
            nchar = snprintf(bufB, NBUFB, "/0D ");
            for (i=0; i < 80; ++i) {
                nchar = snprintf(number_str, 10, "%02x", analog_data_bytes[i]);
                strncat(bufB, number_str, 10);
            }
            nchar = snprintf(number_str, 10, "#\n");
            strncat(bufB, number_str, 10);
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "/0%c error: Unknown command.#\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
    if (!override_led) LED = 0; // To indicate end of interpreter activity.    
} // end interpret_RS485_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200);
    spi1_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        LED = 1;
        __delay_ms(250);
        LED = 0;
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
            char* cmd = trim_RS485_command(bufA, NBUFA);
            // Note that the cmd string may be of zero length,
            // with the null character in the first place.
            // If that is the case, do nothing with it.
            if (*cmd) {
                interpret_RS485_command(cmd);
            }
        }
    }
    uart1_flush_rx();
    spi1_close();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
