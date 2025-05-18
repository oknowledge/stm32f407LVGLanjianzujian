#include "KEY.h"

uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* �������ɿ���־ */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* ֧������ */

    if (key_up && (KEY1 == 1 || KEY2 == 0 || KEY3 == 0 || KEY4 == 0))  /* �����ɿ���־Ϊ 1, ��������һ������������ */
    {
        HAL_Delay(10);          /* ȥ���� */
        key_up = 0;

        if (KEY1 == 1)  keyval = KEY1_PRES;

        if (KEY2 == 0)  keyval = KEY2_PRES;

        if (KEY3 == 0)  keyval = KEY3_PRES;

        if (KEY4 == 0)  keyval = KEY4_PRES;
    }
    else if (KEY1 == 0 && KEY2 == 1 && KEY3 == 1 && KEY4 == 1)         /* û���κΰ�������, ��ǰ����ɿ� */
    {
        key_up = 1;
    }

    return keyval;              /* ���ؼ�ֵ */
}
 