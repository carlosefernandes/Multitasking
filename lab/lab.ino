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

TickType_t xInitialTime;

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
  
        vTaskDelayUntil( &xInitialTime, xFrequency );
        float average = sum / count;
        sum = 0;
        count = 0;
        float R = (1023-average)*10/average;
        //Serial.print("Average "); Serial.print(average); Serial.print("\n");
        //Serial.print("R "); Serial.print(R); Serial.print("\n");
        float lux = 350 * pow(R, -1.43);
        
        Serial.print("Lux "); Serial.print(lux); Serial.print("\n");
    }
}

void TaskPlayMelody( void *pvParameters ) {
    for ( ; ; ) {
        play_melody();       
    }
}

void handleButton() {
    button = 1;
}

ISR(ADC_vect){
 
    // Must read low first
    int analogVal = ADCL | (ADCH << 8);

    count++;
    sum += analogVal;   
}

void setup() {
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(melodyPin, OUTPUT);
    pinMode(interruptVccPin, OUTPUT);
    
    initialize_ADC();

    xInitialTime = xTaskGetTickCount();

    xTaskCreate(TaskDiagnostic, (const portCHAR *)"Diagnostic", 128, NULL, 3 , NULL);
    
    xTaskCreate(TaskOutput, (const portCHAR *)"Output", 128, NULL, 2 , NULL);

    xTaskCreate(TaskPlayMelody, (const portCHAR *)"PlayMelody", 128, NULL, 1 , NULL);

    attachInterrupt(digitalPinToInterrupt(interruptPin), handleButton, RISING);

    digitalWrite(interruptVccPin, HIGH);
}

void loop() {

}

void initialize_ADC() {
  ADMUX &= B11011111;
  ADMUX |= B01000000;
  ADMUX &= B11110000; 
  ADMUX |= 0;
  ADCSRA |= B10000000;
  ADCSRA |= B00100000;  
  ADCSRB &= B11111000;
  ADCSRA |= B00000111;
  ADCSRA |= B00001000;
  sei();
  ADCSRA |=B01000000;
}

