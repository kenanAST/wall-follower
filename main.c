#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define DOT_LENGTH 300
#define TIMEOUT_US 30000 // Timeout in microseconds (adjust as needed)

int trigPin = 12;
int echoPin = 13;

unsigned speedLeft = 0;
unsigned speedRight = 0;

// PID variables
double kp = 5;  // Proportional gain
double ki = 200; // Integral gain
double kd = 0.2;  // Derivative gain
double setpoint = 40.0; // Target distance in centimeters
double error, integral, derivative;
double lastError = 0;


volatile unsigned long millis_count = 0; // Counter for millis-like functionality

void makeOutput(int pin, char value) {
  if (pin >= 8) {
    // For pins 8 and above, use PORTB
    (value) ? (PORTB |= (1 << (pin - 8))) : (PORTB &= ~(1 << (pin - 8)));
  } else {
    // For pins below 8, use PORTD
    (value) ? (PORTD |= (1 << pin)) : (PORTD &= ~(1 << pin));
  }
}

char checkInput(int pin) {
  return (PINB & (1 << (pin - 8)));
}

void initSerial() {
  // Set baud rate to 9600
  UBRR0H = (unsigned char)(103 >> 8);
  UBRR0L = (unsigned char)103;
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // Set frame format: 8 data, 1 stop bit
  UCSR0C = (3 << UCSZ00);
}

void transmitChar(char data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  // Put data into buffer, sends the data
  UDR0 = data;
}

void transmitString(const char* str) {
  while (*str) {
    transmitChar(*str);
    str++;
  }
}

void initTimer() {
  // Set up Timer1 for measuring the pulse duration
  TCCR1B |= (1 << CS11); // Set prescaler to 8
  // Configure Timer0 for PWM on pins 5 and 6
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00); // Set PWM mode and enable non-inverting PWM
  TCCR0B |= (1 << CS01); // Set prescaler to 8
}

unsigned long millis() {
  return millis_count;
}

ISR(TIMER0_COMPA_vect) {
  millis_count++;
}

void initMillisTimer() {
  // Set up Timer0 for millis-like functionality
  TCCR0A |= (1 << WGM01); // Set to CTC mode
  OCR0A = 124;            // Set the compare value for 1ms intervals
  TIMSK0 |= (1 << OCIE0A); // Enable the compare match interrupt
  TCCR0B |= (1 << CS01) | (1 << CS00); // Set prescaler to 64
}

void customAnalogWrite(int pin, int value) {
  // Check if the pin is valid for PWM
  if (pin == 5 || pin == 6) {
    // Set PWM values for pins 5 and 6 using Timer0 and Timer2
    switch (pin) {
      case 5:
        OCR0B = value; // Set PWM value for pin 5 using Timer0
        break;
      case 6:
        OCR0A = value; // Set PWM value for pin 6 using Timer0
        break;
    }
  }
}

int main() {
  initSerial(); // Initialize serial communication
  initTimer();  // Initialize Timer1
  initMillisTimer(); // Initialize Timer0 for millis-like functionality

  DDRB |= (1 << DDB4);   // Trigger Pin to Output;
  DDRB &= ~(1 << DDB5); // Echo Pin to Input;

  DDRD |= (1 << DDD2); 
  DDRD |= (1 << DDD5); 
  DDRD |= (1 << DDD4); 
  DDRD |= (1 << DDD6); 
  

  float duration_us;
  double distance_cm;
  unsigned long lastMillisSensor = 0;
  unsigned long lastMillisLeftMotor = 0;
  unsigned long lastMillisRightMotor = 0;

  sei(); // Enable global interrupts

  while (1) {
    if (millis() - lastMillisSensor >= 50) { // Execute every 100 milliseconds
      lastMillisSensor = millis();

      makeOutput(trigPin, 1);
      _delay_us(10);
      makeOutput(trigPin, 0);

      // Measure the duration of the pulse on the echo pin
      while (!checkInput(echoPin)) {
        // Wait for the echo pulse to start
      }
      TCNT1 = 0; // Reset Timer1 counter
      while (checkInput(echoPin)) {
        // Wait for the echo pulse to end
      }
      duration_us = TCNT1 * 0.5; // Assuming a prescaler of 8, adjust if needed
      distance_cm = duration_us / 58.0;

          // Calculate PID
      error = setpoint - distance_cm;
      integral += error;
      derivative = error - lastError;
      lastError = error;

      speedLeft = constrain(speedLeft + (kp * error + kd * derivative), 80, 150);
      speedRight = constrain(speedRight - (kp * error + kd * derivative), 60, 120);
      

      char buffer[20];
      int distance_int = (int)distance_cm; // Convert floating-point to integer
      snprintf(buffer, sizeof(buffer), "Distance: %d cm\n", distance_int);
      transmitString(buffer);
    }

    if(millis() - lastMillisLeftMotor >= 50){
      makeOutput(2,1);
      customAnalogWrite(5, speedRight);
    }

    if(millis() - lastMillisRightMotor >= 50){
      makeOutput(4,0);
      customAnalogWrite(6, speedLeft);
    }
  }
}