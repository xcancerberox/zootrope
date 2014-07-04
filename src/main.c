#include <avr/io.h>
#include <util/delay.h>
#include "main.h"

uint8_t vel_count;
uint8_t vel_ref;

ISR(INT0_vect){
    // Debounce
    _delay_ms(10);
    if ((SENSOR_HALL_PORT & (1<<SENSOR_HALL_PIN_NUM)) == 0){
        flashLeds();
        vel_count += 1;
    }
}

/*
// vel va entre 0 y PWM_MAX (ej: 500)
// e va entre -PWM_MAX y PWM_MAX
// vel_count 

#define K1 10
void PID(uint8_t e, uint8_t* vel, uint8_t prev_vel){
  *vel = prev_vel + e * (K1/100);
}

ISR(TIMER_OV){
  int e;
  uint8_t vel;
  e = vel_count - vel_ref;
  PID(e,&vel,vel_count);
  setMotorVel(e*k1);
}
*/

void setupHallEffectSensor(void){
  ClearBit(SENSOR_HALL_DDR, SENSOR_HALL_PIN_NUM);
  ClearBit(SENSOR_HALL_PORT, SENSOR_HALL_PIN_NUM);

  vel_count = 0;
  // Configuracion interrupcion externa

  // Configuracion timer con interrupcion para la velocidad
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
  SetBit(FLASH_1_PORT, FLASH_1_PIN_NUM);
}

void flashLeds(void){
  toggle_led_1();
  _delay_ms(FLASH_LEN_MS);
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

void main(void){

  //setupHallEffectSensor();
  setupFlashLeds();
  setupMotor();
  setMotorVel(50);

  for(;;){
    flashLeds();
    _delay_ms(1000);
  }
}
