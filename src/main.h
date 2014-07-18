#ifndef __MAIN_H
#define __MAIN_H

#define FLASH_LEN_MS 100

#define concat(a,b)		a ## b
#define def_port_reg(name)	concat(PORT,name)
#define def_pin_reg(name)	concat(PIN,name)
#define def_ddr_reg(name)	concat(DDR,name)
#define SetBit(Byte,Bit)	 (Byte |= (1<<Bit))
#define	ClearBit(Byte,Bit)	 (Byte &= (~(1<<Bit)))

#define LED_1_PORT_NAME       C
#define LED_1_PIN_NUM         3
#define LED_2_PORT_NAME       C
#define LED_2_PIN_NUM         2
#define LED_3_PORT_NAME       C
#define LED_3_PIN_NUM         1
#define SENSOR_HALL_PORT_NAME B
#define SENSOR_HALL_PIN_NUM   0
#define MOTOR_IN1_PORT_NAME   D
#define MOTOR_IN1_PIN_NUM     2
#define MOTOR_IN2_PORT_NAME   D
#define MOTOR_IN2_PIN_NUM     3
#define MOTOR_EN_PORT_NAME    B
#define MOTOR_EN_PIN_NUM      1 //OC1A
#define FLASH_1_PORT_NAME     D
#define FLASH_1_PIN_NUM       4

#define LED_1_PORT def_port_reg(LED_1_PORT_NAME)
#define LED_1_PIN  def_pin_reg(LED_1_PORT_NAME)
#define LED_1_DDR  def_ddr_reg(LED_1_PORT_NAME)
#define LED_2_PORT def_port_reg(LED_2_PORT_NAME)
#define LED_2_PIN  def_pin_reg(LED_2_PORT_NAME)
#define LED_2_DDR  def_ddr_reg(LED_2_PORT_NAME)
#define LED_3_PORT def_port_reg(LED_3_PORT_NAME)
#define LED_3_PIN  def_pin_reg(LED_3_PORT_NAME)
#define LED_3_DDR  def_ddr_reg(LED_3_PORT_NAME)
#define SENSOR_HALL_PORT def_port_reg(SENSOR_HALL_PORT_NAME)
#define SENSOR_HALL_PIN  def_pin_reg(SENSOR_HALL_PORT_NAME)
#define SENSOR_HALL_DDR  def_ddr_reg(SENSOR_HALL_PORT_NAME)
#define MOTOR_IN1_PORT def_port_reg(MOTOR_IN1_PORT_NAME)
#define MOTOR_IN1_PIN  def_pin_reg(MOTOR_IN1_PORT_NAME)
#define MOTOR_IN1_DDR  def_ddr_reg(MOTOR_IN1_PORT_NAME)
#define MOTOR_IN2_PORT def_port_reg(MOTOR_IN2_PORT_NAME)
#define MOTOR_IN2_PIN  def_pin_reg(MOTOR_IN2_PORT_NAME)
#define MOTOR_IN2_DDR  def_ddr_reg(MOTOR_IN2_PORT_NAME)
#define MOTOR_EN_PORT def_port_reg(MOTOR_EN_PORT_NAME)
#define MOTOR_EN_PIN  def_pin_reg(MOTOR_EN_PORT_NAME)
#define MOTOR_EN_DDR  def_ddr_reg(MOTOR_EN_PORT_NAME)
#define FLASH_1_PORT def_port_reg(FLASH_1_PORT_NAME)
#define FLASH_1_PIN  def_pin_reg(FLASH_1_PORT_NAME)
#define FLASH_1_DDR  def_ddr_reg(FLASH_1_PORT_NAME)

#endif
