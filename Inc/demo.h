#ifndef __demo_h
#define __demo_h

#include "main.h"
#include "lvgl.h"
#include "lv_port_indev_template.h"
#include "lv_port_disp_template.h"
#include "lv_demo_widgets.h"

 
#define scr_act_width() lv_obj_get_width(lv_scr_act())
#define scr_act_height() lv_obj_get_height(lv_scr_act())

void lv_mainstart(void);
//static void lv_example_label_1(void);
//static void lv_example_label_2(void);
//static void lv_example_label_3(void);
static void lv_example_label_4(void);
static void lv_example_label_5(void); 
#endif













