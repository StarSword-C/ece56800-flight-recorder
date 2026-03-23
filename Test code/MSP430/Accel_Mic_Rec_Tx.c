#include <msp430.h>
#include <stdint.h>

/**
 * main.c
 */

/* Program for MSP-EXP430FR2355 to gather accelerometer and microphone data from the
 * TI Educational BoosterPack Mk II, and transmit to an external device by UART.
 *
 * Basically works, but we need to get the sample rate for the microphone much higher
 * to be useful: 16 kHz recommended as Nyquist frequency of the human voice.
 * 
 * Alternatively, just use the accelerometer and reserve the microphone for another
 * sensor device.
 *
 * Accelerometer Pin-outs:
 * - MSP430 P1.4 - Analog 4  -> BP2 J3.23 Accelerometer X
 * - MSP430 P5.3 - Analog 11 -> BP2 J3.24 Accelerometer Y
 * - MSP430 P5.1 - Analog 9  -> BP2 J3.25 Accelerometer Z
 * Microphone Pin-out:
 * - MSP430 P5.2 - Analog 10 -> BP2 J1.6
 * UART pin-outs:
 * - MSP430 P1.7 - ext Tx via eUSCI A0 -> BP2 J1.4
 * - MSP430 P1.6 - ext Rx via eUSCI A0 -> BP2 J1.3
 * - MSP430 P4.3 - USB Tx via eUSCI A1
 * - MSP430 P4.2 - USB Rx via eUSCI A1
 *
 * UART Transmission Order: x, y, z, microphone
 */

/* ---------------- Defines ---------------- */
#define ADC_CH_X   ADCINCH_4    // A4 = Accelerometer X
#define ADC_CH_Y   ADCINCH_11   // A11 = Accelerometer Y
#define ADC_CH_Z   ADCINCH_9    // A9 = Accelerometer Z
#define ADC_CH_MIC ADCINCH_10   // A10 =Microphone


/* ---------------- Globals ---------------- */
// Data from ADC
volatile uint8_t accelerometer_vals[3] = {0, 0, 0};
volatile uint8_t microphone_val = 0;

// UART transmission array and flags
volatile uint8_t tx_buf[4];
volatile uint8_t tx_index_a0 = 0;
volatile uint8_t tx_index_a1 = 0;
volatile uint8_t uart_a0_done = 1;
volatile uint8_t uart_a1_done = 1;

/* which ADC sample are we currently taking?
 * 0 = x, 1 = y, 2 = z, 3 = mic
 */
volatile uint8_t adc_state = 0;


/* ---------------- Function prototypes ---------------- */
static void init_gpio(void);                // configures GPIO ports
static void init_adc(void);                 // configures ADC
static void init_uart_a0(void);             // configures eUSCI 0 for external UART
static void init_uart_a1(void);             // configures eUSCI 1 for USB UART for debugging, disabled in production
static void build_tx_packet(void);          // compiles UART byte stream
static void start_adc_sequence(void);       // starts ADC receive
static void start_uart_transmit(void);      // starts UART send
//static void set_adc_channel(uint8_t state); // cycles through ADC channels DEPRECATED

/* ---------------- Main ---------------- */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;       // disable GPIO power-on default high impedance mode

    init_gpio();
    init_adc();
    init_uart_a0();
    init_uart_a1();             // USB for debugging, disabled in production

    __enable_interrupt();

    while (1)
    {
        start_adc_sequence();
        __bis_SR_register(LPM0_bits | GIE);    // sleep until ADC ISR wakes us

        build_tx_packet();

        uart_a0_done = 0;
        uart_a1_done = 0;                      // USB for debugging, disabled in production

        start_uart_transmit();
        __bis_SR_register(LPM0_bits | GIE);    // sleep until both UARTs finish
    }
}

/* ---------------- Initialization ---------------- */
static void init_gpio(void)     // Configure GPIO
{
    // default all ports to low power first for energy savings (removes ULP warnings)
    P1OUT = 0; P1DIR = 0xFF;
    P2OUT = 0; P2DIR = 0xFF;
    P3OUT = 0; P3DIR = 0xFF;
    P4OUT = 0; P4DIR = 0xFF;
    P5OUT = 0; P5DIR = 0xFF;
    P6OUT = 0; P6DIR = 0xFF;
    PAOUT = 0; PADIR = 0xFF;
    PBOUT = 0; PBDIR = 0xFF;
    PCOUT = 0; PCDIR = 0xFF;

    // ADC input pins
    P1DIR &= ~BIT4;
    P5DIR &= ~(BIT1 | BIT2 | BIT3);

    // select analog function
    P1SEL0 |= BIT4;
    P1SEL1 |= BIT4;

    P5SEL0 |= (BIT1 | BIT2 | BIT3);
    P5SEL1 |= (BIT1 | BIT2 | BIT3);

    // UCA0: P1.6 RX, P1.7 TX
    P1SEL0 |= (BIT6 | BIT7);
    P1SEL1 &= ~(BIT6 | BIT7);

    // UCA1: P4.2 RX, P4.3 TX -- for debugging, disabled in production
    P4SEL0 |= (BIT2 | BIT3);
    P4SEL1 &= ~(BIT2 | BIT3);
}

static void init_adc(void)      // Configure ADC
{
    ADCCTL0 &= ~ADCENC;         // Disable ADC before config

    /* sample-and-hold time, ADC on */
    ADCCTL0 = ADCSHT_2 | ADCON;

    // Set ADC to 8-bit resolution to match UART buffer size
    ADCCTL2 = ADCRES_0;

    /* sampling timer, single-channel single-conversion, use MODOSC */
    ADCCTL1 = ADCSHP | ADCCONSEQ_0 | ADCSSEL_0;

    /* interrupt enable for ADCMEM0 */
    ADCIE = ADCIE0;
}

static void init_uart_a0(void)  // Configure eUSCI 0 for UART (external)
{
    UCA0CTLW0 = UCSWRST;        // Hold eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // SMCLK source

    UCA0BRW = 8;                // transmit at 115.2 kbaud
    UCA0MCTLW = 0xD6;

    UCA0CTLW0 &= ~UCSWRST;      // release for operation
    UCA0IE &= ~UCTXIE;          // TX interrupt off until needed
}

static void init_uart_a1(void)  // Configure eUSCI 1 for UART (USB) -- for debugging, disabled in production
{
    UCA1CTLW0 = UCSWRST;        // second verse, same as the first
    UCA1CTLW0 |= UCSSEL__SMCLK;

    UCA1BRW = 8;
    UCA1MCTLW = 0xD6;

    UCA1CTLW0 &= ~UCSWRST;
    UCA1IE &= ~UCTXIE;
}

/* ---------- Helpers ---------- */
static void build_tx_packet(void)       // creates transmission bytestream for UART
{
    /*
     * Big-endian word order in the byte stream:
     * X, Y, Z, Mic
     */
    tx_buf[0] = (uint8_t)accelerometer_vals[0];
    tx_buf[1] = (uint8_t)accelerometer_vals[1];
    tx_buf[2] = (uint8_t)accelerometer_vals[2];
    tx_buf[3] = (uint8_t)microphone_val;
}

/* DEPRECATED: had been called from start_adc_sequence() and ADC_ISR() in early test versions
 * but threw ULP warning in the latter.
static void set_adc_channel(uint8_t state)  // cycles through reading ADC channels
{
    ADCCTL0 &= ~ADCENC;

    switch (state)
    {
    case 0:
        ADCMCTL0 = ADCINCH_4;   // P1.4 = A4 = accel x
        break;
    case 1:
        ADCMCTL0 = ADCINCH_11;  // P5.3 = A11 = accel y
        break;
    case 2:
        ADCMCTL0 = ADCINCH_9;   // P5.1 = A9 = accel z
        break;
    case 3:
        ADCMCTL0 = ADCINCH_10;  // P5.2 = A10 = mic
        break;
    default:
        ADCMCTL0 = ADCINCH_4;
        break;
    }
}*/

static void start_adc_sequence(void)    // restarts ADC after configuration
{
    adc_state = 0;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 = ADC_CH_X;                // set to Accel X
    ADCCTL0 |= ADCENC | ADCSC;          // Enable and start conversion
}

static void start_uart_transmit(void)   // sends bytestream by UART
{
    tx_index_a0 = 0;
    tx_index_a1 = 0;                    // USB for debugging; disabled in production

    /*
     * Prime both UARTs with first byte, then let TX interrupts send the rest.
     * Comment out UCA1 lines in production if desired.
     */
    UCA0TXBUF = tx_buf[tx_index_a0++];
    UCA0IE |= UCTXIE;

    UCA1TXBUF = tx_buf[tx_index_a1++];
    UCA1IE |= UCTXIE;
}

/* ---------- ADC ISR ---------- */
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch (__even_in_range(ADCIV, ADCIV__ADCIFG0))
    {
    case ADCIV__NONE:
        break;

    case ADCIV__ADCIFG0:
        switch (adc_state)
        {
        case 0:
            accelerometer_vals[0] = (uint8_t)ADCMEM0; // read accel X from ADC

            adc_state++;
            ADCCTL0 &= ~ADCENC;
            ADCMCTL0 = ADC_CH_Y;   // advance to accel Y for next conversion (P5.3)
            ADCCTL0 |= ADCENC | ADCSC;
            break;

        case 1:
            accelerometer_vals[1] = (uint8_t)ADCMEM0; // read accel Y

            adc_state++;
            ADCCTL0 &= ~ADCENC;
            ADCMCTL0 = ADC_CH_Z;    // advance to accel Z (P5.1)
            ADCCTL0 |= ADCENC | ADCSC;
            break;

        case 2:
            accelerometer_vals[2] = (uint8_t)ADCMEM0; // read accel Z

            adc_state++;
            ADCCTL0 &= ~ADCENC;
            ADCMCTL0 = ADC_CH_MIC;   // advance to Mic (P5.2)
            ADCCTL0 |= ADCENC | ADCSC;
            break;

        case 3:
            microphone_val = (uint8_t)ADCMEM0; // read Mic

            /* Done with sequence */
            __bic_SR_register_on_exit(LPM0_bits);
            break;

        default:
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        }
        break;

    default:
        break;
    }
}

/* ---------- UART A0 ISR ---------- */
#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_ISR(void)
{
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;

    case USCI_UART_UCTXIFG:
        if (tx_index_a0 < sizeof(tx_buf))
        {
            UCA0TXBUF = tx_buf[tx_index_a0++];
        }
        else
        {
            UCA0IE &= ~UCTXIE;
            uart_a0_done = 1;

            if (uart_a1_done)
            {
                __bic_SR_register_on_exit(LPM0_bits);
            }
        }
        break;

    default:
        break;
    }
}

/* ---------- UART A1 ISR ---------- */
#pragma vector=EUSCI_A1_VECTOR
__interrupt void EUSCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;

    case USCI_UART_UCTXIFG:
        if (tx_index_a1 < sizeof(tx_buf))
        {
            UCA1TXBUF = tx_buf[tx_index_a1++];
        }
        else
        {
            UCA1IE &= ~UCTXIE;
            uart_a1_done = 1;

            if (uart_a0_done)
            {
                __bic_SR_register_on_exit(LPM0_bits);
            }
        }
        break;

    default:
        break;
    }
}
