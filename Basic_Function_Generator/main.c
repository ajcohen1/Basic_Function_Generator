#include <stdio.h>
#include <math.h>
#include <float.h>
#include "msp.h"
#include "KEYPAD_DRIVER.h"

#define GAIN BIT5
#define SHDN BIT4
#define DAC_CS  BIT4
#define MAX 4096
#define MIN 2483

typedef enum {SQUARE, SIN, SAWTOOTH} waveType ;

//These are the default values. Made global for easier communication with ISR
waveType wt = SQUARE;
unsigned dt = 50;
unsigned freq = 100;

void SPI_init() {
    P1->SEL0 |= BIT5 | BIT6 | BIT7;     // Set P1.5, P1.6, and P1.7 as
                                        // SPI pins functionality

    P2->DIR |= DAC_CS;                      // set as output for CS
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put eUSCI state machine in reset

    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST    |  // keep eUSCI in reset
                      EUSCI_B_CTLW0_MST      |  // Set as SPI master
                      EUSCI_B_CTLW0_SYNC     |  // Set as synchronous mode
                      EUSCI_B_CTLW0_CKPL     |  // Set clock polarity high
                      EUSCI_B_CTLW0_UCSSEL_2 |  // SMCLK
                      EUSCI_B_CTLW0_MSB;        // MSB first

    EUSCI_B0->BRW = 0x02;              // div by 2 fBitClock = fBRCLK / UCBRx

    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;  // Initialize USCI state machine
}

int freq_to_CCR_3mhz(unsigned frequency) {
    return 3000000/frequency;
}

void SPI_t_DAC(uint16_t data) {
    uint8_t hiByte, loByte;
    // set the low and high bytes of data
              loByte = 0xFF & data;         // mask just low 8 bits
              hiByte = 0x0F & (data >> 8);  // shift and mask bits for D11-D8
              hiByte |= (GAIN | SHDN);      // set the gain / shutdown control bits

              P2->OUT &= ~DAC_CS; // Set CS low

              // wait for TXBUF to be empty before writing high byte
              while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
              EUSCI_B0->TXBUF = hiByte;

              // wait for TXBUF to be empty before writing low byte
              while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
              EUSCI_B0->TXBUF = loByte;

              // wait for RXBUF to be empty before changing CS
              while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));

              P2->OUT |= DAC_CS; // Set CS high
}

void timer_init() {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // clear interrupt


    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled

    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1; // SMCLK, UP mode

    TIMER_A0->CCR[0] = freq_to_CCR_3mhz(freq);

    NVIC_SetPriority(TA0_0_IRQn, 2);
    NVIC_EnableIRQ(TA0_0_IRQn);  // set NVIC interrupt

    __enable_irq();     // Enable global interrupt
}

int count = 0;
void displaySquareWave() {
    count ^= 1;
    int ccr_val = 0;
    if(count) { //time on
        SPI_t_DAC(MAX-1);
        ccr_val = dt * freq_to_CCR_3mhz(freq) / 100;
        TIMER_A0->CCR[0] = ccr_val;
    }
    else {  //time off
        SPI_t_DAC(MIN);
        ccr_val = (100-dt) * freq_to_CCR_3mhz(freq) / 100;
        TIMER_A0->CCR[0] = ccr_val;
    }
}

int get_step_size() {
    switch(freq) {
    case 100:
        return 6;
    case 200:
        return 12;
    case 300:
        return 24;
    case 400:
        return 48;
    case 500:
        return 96;
    }
    return -1;
}

void displaySawtoothWave() {
    unsigned data;
    int ss = get_step_size();
    for(data = MIN; data < MAX; data += ss)
        SPI_t_DAC(data);
    TIMER_A0->CCR[0] = 1;
}

int sin100[268];
int sin200[134];
int sin300[67];
int sin400[33];
int sin500[16];
int sinTables[5][268];


void load_single_sine_table() {
    unsigned ndx = 0, num_points = ((MAX-MIN) / get_step_size())+1, table_ndx;
    double period = 1.0/freq, time_ss = 2 * period / num_points, time, test;
    for(ndx = 0, time = 0; ndx < num_points-1;
                time += time_ss, ndx++) {
        test = sin(M_PI * freq * time);
        table_ndx = (freq/100)-1;
        sinTables[(freq/100)-1][ndx] = MIN + (MAX- MIN - 1) * sin(M_PI * freq * time);
    }
}

void load_sine_tables() {
    unsigned ndx;
    for(ndx = 0; ndx < 5; ndx++) {
        freq = (ndx+1)*100;
        load_single_sine_table();
    }
    freq = 100;

}

void displaySinWave() {
    unsigned ndx, num_points = (MAX-MIN) / get_step_size();
    for(ndx = 0; ndx < num_points; ndx++)
        SPI_t_DAC(sinTables[(freq/100)-1][ndx]);
    TIMER_A0->CCR[0] = 1;
}

void displayWave() {
    switch(wt){
    case SQUARE: displaySquareWave();
                 break;
    case SAWTOOTH: displaySawtoothWave();
                 break;
    case SIN: displaySinWave();
                 break;
    }
}

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // Clear the CCR0 interrupt
    displayWave();
}

void keypad_interrupt_init() {
    P4->IES = 0x00;        // Interrupt on high-to-low transition
    P4->IFG = 0x00;       // Clear all P1 interrupt flags
    P4->IE = 0x7F;        // Enable interrupt for all port 1
    NVIC_SetPriority(PORT4_IRQn, 0);
    NVIC_EnableIRQ(PORT4_IRQn);  // set NVIC interrupt
}

void init_devices() {
    keypad_interrupt_init();
    keypad_init();
    SPI_init();
    timer_init();
    load_sine_tables();
}

void modify_state(uint8_t key) {
    if(key < 7 || key > 9)
        return;

    switch(key) {
    case 7: wt = SQUARE;
            break;
    case 8: wt = SIN;
            break;
    case 9: wt = SAWTOOTH;
            break;
    }
}

void modify_freq(uint8_t key) {
    if(key < 1 || key > 5)
        return;

    switch(key) {
    case 1: freq = 100;
            break;
    case 2: freq = 200;
            break;
    case 3: freq = 300;
            break;
    case 4: freq = 400;
            break;
    case 5: freq = 500;
            break;
    }
}

void modify_dt(uint8_t key) {
    if(key != 0 && key != 10 && key != 12)
        return;

    switch(key) {
    case 10: dt -= 10;
             break;
    case 12: dt += 10;
             break;
    case 0:  dt = 50;
             break;
    }

    if(dt > 90)
        dt = 90;
    else if(dt < 10)
        dt = 10;

}

void modify_attributes(uint8_t key) {
    modify_state(key);
    modify_freq(key);
    modify_dt(key);
}

void delay() {
    unsigned ndx;
    for(ndx = 0; ndx < 100000; ndx++);
}

void PORT4_IRQHandler(void) {
    uint8_t key;
    P4->IFG = 0x00;
    key = keypad_getkey();
    delay();
    printf("%d\n", key);
    modify_attributes(key);
}

void main(void)
{
    init_devices();
    load_sine_tables();

    while(1);
}
