#include "mario.h"
#include <Arduino_FreeRTOS.h>
#include <stdio.h>

#define melodyPin 3

volatile long count = 0;
volatile long sum = 0;
volatile char button = 0;

const byte interruptPin = 2;
const byte interruptVccPin = 7;

const TickType_t xDebounceDelay = 200 / portTICK_PERIOD_MS;
const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

TickType_t xLastWakeTime;

void initialize_ADC();

void TaskDiagnostic( void *pvParameters ) {
    for ( ; ; ) {
        vTaskDelay( xDebounceDelay );
        if (button == 1) {
            vTaskDelay( xDebounceDelay );
            //char diagnostic[30];
            //sprintf(diagnostic, "Sum %ld Count %ld\n", sum, count);
            //Serial.write(diagnostic);
            button = 0;
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}
 
void TaskOutput( void *pvParameters ) {
    const TickType_t xFrequency = 5000 / portTICK_PERIOD_MS;
    for ( ; ; ) {
  
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        float average = sum / count;
        sum = 0;
        count = 0;
        long resistence = (1023 - average) * 10 / average;
        long lux = 0;//350 * pow(resistence, -1.43);
        char output[50];
        sprintf(output, "Lux %ld resistence %f\n", lux, resistence);
        Serial.write(output);
        
    }
}

void TaskPlayMelody( void *pvParameters ) {
    for ( ; ; ) {
        //Serial.print("playMelody");
        play_melody();       
    }
}

void handleButton() {
    button = 1;
    Serial.print("handleButton\n");
}

ISR(ADC_vect){
 
    // Must read low first
    int analogVal = ADCL | (ADCH << 8);

    count++;
    sum += analogVal;
/*
    Serial.print("Count: ");
    Serial.print(count);
    Serial.print("\n");
/*    Serial.print(sum);*/
    
}

void setup() {
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(melodyPin, OUTPUT);
    pinMode(interruptVccPin, OUTPUT);
    
    initialize_ADC();

    xLastWakeTime = xTaskGetTickCount();

    xTaskCreate(TaskDiagnostic, (const portCHAR *)"Diagnostic", 128, NULL, 3 , NULL);
    
    xTaskCreate(TaskOutput, (const portCHAR *)"Output", 128, NULL, 2 , NULL);

    xTaskCreate(TaskPlayMelody, (const portCHAR *)"PlayMelody", 128, NULL, 1 , NULL);

    attachInterrupt(digitalPinToInterrupt(interruptPin), handleButton, RISING);

    digitalWrite(interruptVccPin, HIGH);
}

void loop() {

}

void initialize_ADC() {
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  ADMUX |= 0;
  // ADMUX |= B00001000; // Binary equivalent
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;
  
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;
  
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000111;
  
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;
}

