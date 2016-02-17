/*
 * ATTiny25 Temperature Controlled Relay
 * Relay turns on at 45C
 * Relay Turns off at 40C
 * ------------------------------- 
 * Connections 
 * 2.7 - 5 Volts to      Pin 8
 * Ground to             Pin 4 
 * Long Led lead to      Pin 5
 * 1K Resistor           Short LED leg
 * 1K Resistor to        Pin 4
 * Relay Control to      Pin 6
 * Relay VCC to          Pin 8
 * Relay GND to          Pin 4
 * LM35 VCC to           Pin 8
 * LM35 GND to           Pin 4
 * LM35 DATA to          
 
 * ATTiny25 Pin Layout
 *            _____
 *  RESET   1 |*  | 8   VCC
 *  A3/D3   2 |   | 7   A1/D2
 *  A2/D4   3 |   | 6   PWM/D1
 *  GND     4 |___| 5   PWM/D0
 *            
 *
 *
 * LM35 Pin Layout
 * Flat side facing you
 *
 *   _______ GND
 *  |   |___ DATA
 *  |___|___ VCC
 *
 *
 
 *  -------------------------------
 */


// Library Imports for low power mode
#include <avr/sleep.h>
#include <avr/interrupt.h>

// LED Pin Connection
#define LED 0
// Relay Control Pin Connection
#define RELAY 1
// Relay on Temperature
#define RELAY_ON 45
// Relay off temperature
#define RELAY_OFF 40

// Routines to set and claer bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Determines if Watchdog Timer is still counting
volatile boolean f_wdt = 1;
// LM35 Temperature Data Connection
const int LM35 = A3;
// Converts LM35 mV data to Degrees 
const float RATIO = 0.48828125;

void setup()
{
  // Setup LED Connection
  pinMode(LED,OUTPUT);
  // Setup Relay Connection
  pinMode(RELAY,OUTPUT);
  // Turn On LED
  digitalWrite(LED,HIGH);
  // Wait 500ms
  delay(500);
  // Turn Off LED
  digitalWrite(LED,LOW);
  // Watchdog Timings 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms
  // 5=500ms 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  setup_watchdog(9); // approximately 8 seconds sleep  
}

void loop()
{
  // If Watchdog has expired run this code
  if (f_wdt==1) 
  {
    // Reset Watchdog flag
    f_wdt=0;
    // Get mV reading from LM35
    float temp = analogRead(LM35);
    // Convert to Degrees
    temp = temp * RATIO;
    // If temperature above RELAY_ON run this code
    if (temp > RELAY_ON)
    {   
      // While temperature remains above RELAY_OFF run this code
      while(temp > RELAY_OFF)
      {
        // Turn on Relay
        digitalWrite(RELAY,HIGH);
        // Wait 4.5s
        delay(4500);
        // Turn on LED
        digitalWrite(LED,HIGH);
        // Wait 500ms
        delay(500);
        // Turn Off LED
        digitalWrite(LED,LOW);
      }
      // Turn Off Relay
      digitalWrite(RELAY,LOW);
    }
    // Enter Low Power Mode for 8 seconds
    system_sleep();
  }  
}

// Micro Controller Sleep Function
void system_sleep() 
{
  // switch Analog to Digitalconverter OFF
  cbi(ADCSRA,ADEN);               
  // sleep mode is set here
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  // Enable Sleep mode
  sleep_enable();
  // System sleeps here
  sleep_mode();        
  // System continues execution here when watchdog timed out
  sleep_disable();                     
  // switch Analog to Digitalconverter ON
  sbi(ADCSRA,ADEN);                    
}

// Initial Setup of WatchDog Timer
void setup_watchdog(int ii) 
{
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) 
{
  f_wdt=1;  // set global flag
}
