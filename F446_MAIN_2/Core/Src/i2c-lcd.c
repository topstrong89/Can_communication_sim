/*
 * i2c-lcd.c
 *
 *  Created on: Mar 12, 2026
 *      Author: Administrator
 */
#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c2;

#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd(char cmd) {
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C;  // en=1, rs=0
  data_t[1] = data_u | 0x08;  // en=0, rs=0
  data_t[2] = data_l | 0x0C;  // en=1, rs=0
  data_t[3] = data_l | 0x08;  // en=0, rs=0
  HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data) {
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;  // en=1, rs=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1
  data_t[2] = data_l | 0x0D;  // en=1, rs=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1
  HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_init(void) {
  HAL_Delay(50);
  lcd_send_cmd(0x30); HAL_Delay(5);
  lcd_send_cmd(0x30); HAL_Delay(1);
  lcd_send_cmd(0x30); HAL_Delay(10);
  lcd_send_cmd(0x20); HAL_Delay(10);
  lcd_send_cmd(0x28); HAL_Delay(1);
  lcd_send_cmd(0x08); HAL_Delay(1);
  lcd_send_cmd(0x01); HAL_Delay(2); // Clear display
  lcd_send_cmd(0x06); HAL_Delay(1);
  lcd_send_cmd(0x0C); // Display ON, Cursor OFF
}

void lcd_send_string(char *str) {
  while (*str) lcd_send_data(*str++);
}

void lcd_put_cur(int row, int col) {
    switch (row) {
        case 0: col |= 0x80; break;
        case 1: col |= 0xC0; break;
        case 2: col |= 0x94; break;
        case 3: col |= 0xD4; break;
    }
    lcd_send_cmd(col);
}

void lcd_clear(void) {
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

