/*
 * lcd.h
 *
 *  Created on: 18 Nov 2017
 *      Author: choirul anam
 */

#ifndef LCD_H_
#define LCD_H_
#include "stdint.h"

#define D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_8
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_5
#define EN_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_4
#define RW_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_3
#define RS_GPIO_Port GPIOB
void enable(void);
void PORT(uint16_t count);
void lcd_init(void);
void lcd_clear(void);
void lcd_gotoxy(char i,char j);
void lcd_putchar(unsigned char kar);
void lcd_putstr(char *s);

#endif /* LCD_H_ */
