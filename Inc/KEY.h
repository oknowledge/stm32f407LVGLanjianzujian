#ifndef __KEY_H
#define __KEY_H
#include "main.h"
#define KEY1 HAL_GPIO_ReadPin (KEY1_GPIO_Port, KEY1_Pin) //��ȡ KEY1 ���� /
#define KEY2 HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) //��ȡ KEY2 ���� /
#define KEY3 HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) //��ȡ KEY3 ���� /
#define KEY4 HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) //��ȡ KEY4 ���� */
#define KEY1_PRES 1 // KEY1 ���� /
#define KEY2_PRES 2 //KEY2 ���� /
#define KEY3_PRES 3 //KEY3 ���� /
#define KEY4_PRES 4 //KEY4 ���� */
uint8_t key_scan (uint8_t mode); /* ����ɨ�躯�� */
#endif