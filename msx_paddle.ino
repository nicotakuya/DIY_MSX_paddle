// MSX Paddle controller
// for Arduino ATmega328p 5V 16MHz 
// by takuya matsubara
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define CPUHZ 16000000  // CPU frequency[Hz]
#define PRS1  1         // timer1 prescaler
#define PRS2  128       // timer2 prescaler
#define T1HZ  (CPUHZ/PRS1)  // timer1 freq.[Hz]
#define T2HZ  (CPUHZ/PRS2)  // timer2 freq.[Hz]

#define TCNT2_1MSEC     (unsigned int)(T2HZ/1000)
#define TCNT2_1100USEC  (unsigned int)(T2HZ/909)
#define TIMERTEMP      TCNT2
#define TIMERLIMIT     TCNT2_1MSEC

#define TIMER_250NSEC  (unsigned int)(0x10000-(T1HZ/4000000))
#define TIMER_500NSEC  (unsigned int)(0x10000-(T1HZ/2000000))
#define TIMER_1USEC    (unsigned int)(0x10000-(T1HZ/1000000))
#define TIMER_2USEC    (unsigned int)(0x10000-(T1HZ/500000))
#define TIMER_4USEC    (unsigned int)(0x10000-(T1HZ/250000))
#define TIMER_6USEC    (unsigned int)(0x10000-(T1HZ/166666))
#define TIMER_10USEC   (unsigned int)(0x10000-(T1HZ/100000))
#define TIMER_12USEC   (unsigned int)(0x10000-(T1HZ/83333))
#define TIMER_22USEC   (unsigned int)(0x10000-(T1HZ/45454))
#define TIMER_50USEC   (unsigned int)(0x10000-(T1HZ/20000))
#define TIMER_100USEC  (unsigned int)(0x10000-(T1HZ/10000))
#define TIMER_1MSEC    (unsigned int)(0x10000-(T1HZ/1000))

// timer initialize
void timer_init(void)
{
  // timer1 prescaler
  TCCR1A = 0;
  TCCR1B = 1;
  // 0: No clock source (Timer/Counter stopped).
  // 1: clock /1 (No prescaling)
  // 2: clock /8 (From prescaler)
  // 3: clock /64 (From prescaler)
  // 4: clock /256 (From prescaler)
  // 5: clock /1024 (From prescaler)

  // timer2 prescaler
  TCCR2A = 0;
  TCCR2B = 5;
  // 1: clock /(No prescaling)
  // 2: clock /8 (From prescaler)
  // 3: clock /32 (From prescaler)
  // 4: clock /64 (From prescaler)
  // 5: clock /128 (From prescaler)
  // 6: clock /256 (From prescaler)
  // 7: clock /1024 (From prescaler)
}

//----- wait micro second
void timer_uswait(unsigned int limitcnt)
{
  TCNT1 = limitcnt;
  TIFR1 |= (1 << TOV1);  // clear TOV1
  while(!(TIFR1 & (1 << TOV1)));
}

//----- wait mili second
void timer_delay(int milisec)
{
  while(milisec--){
    TCNT2 = 0;
    while(TCNT2 < TCNT2_1MSEC);
  }
}

//---- ADC
//---- AD init
void adc_init(void)
{
//#define ADCLOCK  0 // clock 1/2 
//#define ADCLOCK  1 // clock 1/2 
#define ADCLOCK  2 // clock 1/4 
//#define ADCLOCK  3 // clock 1/8 
//#define ADCLOCK  4 // clock 1/16 
//#define ADCLOCK  5 // clock 1/32 
//#define ADCLOCK  6 // clock 1/64 
//#define ADCLOCK  7 // clock 1/128     
  DDRC &= ~((1<<1)|(1<<0));   // input
  PORTC &= ~((1<<1)|(1<<0));  // no pull up
  ADCSRA = (1<<ADEN)|(0<<ADIE)|(1<<ADIF) | ADCLOCK;
// ADEN: ADC enable
// ADIF: ADC interrupt flag
// ADIE: ADC interrupt enable 1=enable/0=disable
  ADCSRB = 0;
}

//---- AD start
unsigned char adc_get(char adchan)
{
  int tempcnt;
  ADMUX = (1<<REFS0) | adchan; // select AD channel
  timer_uswait(TIMER_10USEC);
  ADCSRA |= (1<<ADSC);  // AD start
  while((ADCSRA & (1<<ADIF))==0){
  }
  tempcnt = (int)ADCL; 
  tempcnt += ((int)ADCH) << 8;
  return((unsigned char)(tempcnt >> 2));
}

// MD/X68K
#define BITTXRX      0b11
#define MD_PORT      PORTD   // Data/LH/ACK port
#define MD_DDR       DDRD    // Data/LH/ACK direction
#define MD_PIN       PIND    // Data/LH/ACK Pin
#define MD_BITALL   (0b111111<<2)  // ALL mask

#define X68K_BITUP    (1<<2)  // up mask
#define X68K_BITDOWN  (1<<3)  // down mask
#define X68K_BITLEFT  (1<<4)  // left mask
#define X68K_BITRIGHT (1<<5)  // right mask
#define X68K_BITA     (1<<6)  // a mask
#define X68K_BITB     (1<<7)  // b mask

#define REQ_PORT PORTB  // REQ port
#define REQ_PIN PINB    // REQ pin
#define REQ_DDR DDRB    // REQ direction
#define REQ_BIT (1<<0)  // REQ mask
#define UNTIL_REQ_H while((REQ_PIN&REQ_BIT)==0)
#define UNTIL_REQ_L while((REQ_PIN&REQ_BIT)!=0)

//------ MD port initialize
void mdport_init(void)
{
  REQ_DDR &= ~REQ_BIT;  // direction input
  REQ_PORT |= REQ_BIT;  // pull up
  MD_DDR |= MD_BITALL;  // direction output
  MD_PORT |= MD_BITALL; // output high
}

//---- appendix:MSX arkanoid vol edition
// please add this parts
//  PORTC0:Player Volume(100k ohm B curve)
//  PORTC2:Player button
void msx_arkanoid_vol(void)
{
#define VOL_PORT PORTC
#define VOL_DDR  DDRC
#define VOL_PIN  PINC
#define VOL_BITBTN  (1<<2) // player button
#define ARKA_BITDAT   X68K_BITUP  
#define ARKA_BITBTN   X68K_BITDOWN
#define ARKA_BITCLK   X68K_BITA
  unsigned char loopcnt;
  unsigned char senddata;

  mdport_init();
  MD_DDR &= ~ARKA_BITCLK;   // direction input
  VOL_DDR &= ~VOL_BITBTN;   // direction input
  VOL_PORT |= VOL_BITBTN;   // pull up
  cli();
  while(1){
    UNTIL_REQ_L;
    UNTIL_REQ_H;
    if((VOL_PIN & VOL_BITBTN)==0){  // button ON
      MD_PORT &= ~ARKA_BITBTN;
    }else{
      MD_PORT |= ARKA_BITBTN;
    }
    senddata = adc_get(0); // player1 Volume
    loopcnt = 8;
    while(loopcnt--){
      if(0x80 & senddata){
        MD_PORT |= ARKA_BITDAT;
      }else{
        MD_PORT &= ~ARKA_BITDAT;
      }
      // until CLK=low
      while((MD_PIN & ARKA_BITCLK)!=0){
        if((REQ_PIN & REQ_BIT)==0)break;
      }
      if((REQ_PIN & REQ_BIT)==0)break;
      while((MD_PIN & ARKA_BITCLK)==0); // until CLK=high
      senddata <<= 1;
    }
    MD_PORT &= ~ARKA_BITDAT;
  }
}

//----
void setup()
{
  timer_init();
  adc_init();
  msx_arkanoid_vol();
}

void loop() {
}
