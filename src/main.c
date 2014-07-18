#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "main.h"

uint8_t vel_count;
uint8_t vel_ref;

void setupHallEffectSensor(void){
  ClearBit(SENSOR_HALL_DDR, SENSOR_HALL_PIN_NUM);
  SetBit(SENSOR_HALL_PORT, SENSOR_HALL_PIN_NUM);

  // Configuracion interrupcion externa
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 to trigger an interrupt on state change 
}

#define PRESCALER_PWM_OFF	(0<<CS12)|(0<<CS11)|(0<<CS10)
#define PRESCALER_PWM_ON	(0<<CS12)|(0<<CS11)|(1<<CS10)
#define MAX_PWM 500UL
void setupMotor(void){
    SetBit(MOTOR_IN1_DDR, MOTOR_IN1_PIN_NUM);
    SetBit(MOTOR_IN2_DDR, MOTOR_IN2_PIN_NUM);
    SetBit(MOTOR_EN_DDR, MOTOR_EN_PIN_NUM);
    SetBit(MOTOR_IN1_PORT, MOTOR_IN1_PIN_NUM);
    ClearBit(MOTOR_IN2_PORT, MOTOR_IN2_PIN_NUM);
    SetBit(MOTOR_EN_PORT, MOTOR_EN_PIN_NUM);

    // Configurar PWM en fase and frec correct
	// Configuracion del timer 1 para el PWM
	TCCR1A = ( (1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10) );
	TCCR1B = ( (1<<WGM13) | (1<<WGM12) | PRESCALER_PWM_ON );

    //(F_CPU/PWM_FREC) Calcula ICR1
	ICR1 = MAX_PWM;  // Esto define al TOP
	//Habilitamos la interrupcion del Timer1 del overflow
	//TIMSK1 = ( (1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) );
}

void setMotorVel(uint8_t vel){
  // Cambiar nivel de comparacion del pwm
  OCR1A = (MAX_PWM/100)*vel;
}

void motorStartMove(void){
  uint8_t i;
  uint8_t vel;
  for(i=0; i < (2^16); i++){
    vel = i;
    setMotorVel(vel);
    _delay_ms(100);
  }
}

// Flash
uint8_t flash_leds_list[5];

void toggle_led_1(void){
  SetBit(FLASH_1_PIN, FLASH_1_PIN_NUM);
}

void setupFlashLeds(void){
  SetBit(FLASH_1_DDR, FLASH_1_PIN_NUM);
  ClearBit(FLASH_1_PORT, FLASH_1_PIN_NUM);
}

#define AD1_PIN_NUM 4
#define AD2_PIN_NUM 5
#define AD1_MUX 4
#define AD2_MUX 5
uint8_t ad1_var;
uint8_t ad2_var;

void flashLeds(void){
  toggle_led_1();
  if(ad2_var < 20){
        _delay_ms(20);
  } else if (ad2_var < 40){
        _delay_ms(40);
  } else if (ad2_var < 60){
        _delay_ms(60);
  } else if (ad2_var < 80){
        _delay_ms(80);
  } else if (ad2_var < 100){
        _delay_ms(100);
  } else if (ad2_var < 120){
        _delay_ms(120);
  } else if (ad2_var < 140){
        _delay_ms(140);
  } else if (ad2_var < 160){
        _delay_ms(160);
  } else if (ad2_var < 180){
        _delay_ms(180);
  } else {
        _delay_ms(200);
  }
  toggle_led_1();
}

void setupDebugLeds(void){
  SetBit(LED_1_DDR, LED_1_PIN_NUM);
  SetBit(LED_2_DDR, LED_2_PIN_NUM);
  SetBit(LED_3_DDR, LED_3_PIN_NUM);
  SetBit(LED_1_PORT, LED_1_PIN_NUM);
  SetBit(LED_2_PORT, LED_2_PIN_NUM);
  SetBit(LED_3_PORT, LED_3_PIN_NUM);
}

void setupAD(void){
    ClearBit(DDRC, AD1_PIN_NUM);
    ClearBit(DDRC, AD2_PIN_NUM);
    SetBit(PORTC, AD1_PIN_NUM);
    SetBit(PORTC, AD2_PIN_NUM);

    ADMUX =_BV(ADLAR);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

void readAD(void){
    ADMUX =_BV(ADLAR)+AD1_MUX;
    ADCSRA |=_BV(ADSC);
    while(ADCSRA &_BV(ADSC)) {}
    ad1_var = ADCH;

    ADMUX =_BV(ADLAR)+AD2_MUX;
    ADCSRA |=_BV(ADSC);
    while(ADCSRA &_BV(ADSC)) {}
    ad2_var = ADCH;
}

void main(void){

  setupHallEffectSensor();
  setupAD();
  setupFlashLeds();
  setupMotor();
  setMotorVel(70);
  sei();

  for(;;){
    //flashLeds();
    readAD();
    setMotorVel(ad1_var*100/256);
    _delay_ms(1000);
  }
}


ISR(PCINT0_vect){
    _delay_ms(10);
    if( (PINB & (1 << PINB0)) == 0 ){
        flashLeds();
    }
}
