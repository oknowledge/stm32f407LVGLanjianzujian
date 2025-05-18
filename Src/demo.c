/**************************** 第一部分 开始 ****************************/ 
/** 
* @brief  LVGL 演示 
253 
 
正点原子 LVGL开发教程 
 254 
LVGL开发指南 
 * @param  无 
 * @return 无 
 */ 
#include "demo.h"
LV_FONT_DECLARE(font20);
void lv_mainstart(void) 
{ 
//    lv_example_label_1(); 
//    lv_example_label_2(); 
//    lv_example_label_3(); 
	  lv_example_label_4();
	  lv_example_label_5(); 	
} 
/**************************** 第一部分 结束 ****************************/ 
 
/**************************** 第二部分 开始 ****************************/ 
/** 
 * @brief  例1 
 * @param  无 
 * @return 无 
 */ 
static void lv_example_label_4(void) 
{ 

 /* 定义并创建标签 */ 
lv_obj_t* label = lv_label_create(lv_scr_act());  
/* 设置标签文本 */          
lv_label_set_text(label, "#0000ff 无锡太湖学院# #ff00ff 物联网2202#" "\n" 
                             "#ff00aa 赵荣胜 #" 
                             "#aa00ff 小阿巴老祖 #"); 
 /* 启用标签文本重新着色 */ 
lv_label_set_recolor(label, true);       
/* 设置标签文本字体 */  
lv_obj_set_style_text_font(label, &font20, LV_PART_MAIN);  
/* 设置标签宽度 */ 
//lv_obj_set_width(label, scr_act_width() );           
/* 设置标签位置 */ 
lv_obj_align(label, LV_ALIGN_CENTER, -scr_act_width() / 16, 0);  
/* 设置标签文本对齐方式 */ 
lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);   
} 

static void lv_example_label_5(void) 
{
//    // 创建一个普通标签
//    lv_obj_t* label = lv_label_create(lv_scr_act());  
//    // 启用标签文本重新着色
//    lv_label_set_recolor(label, true);       
//    // 设置标签文本字体
//    // 注意：需要确保 font20 已经正确定义
//    lv_obj_set_style_text_font(label, &font20, LV_PART_MAIN);  

    // 创建一个带有符号的标签
    lv_obj_t *sym_label = lv_label_create(lv_scr_act());
    // 设置标签文本
    lv_label_set_text(sym_label, "5G" LV_SYMBOL_WIFI);
    // 设置标签位置
    lv_obj_set_pos(sym_label, 430, 0);
}
//static void lv_example_label_1(void) 
//{ 

// /* 定义并创建标签 */ 
//lv_obj_t* label = lv_label_create(lv_scr_act());  
///* 设置标签文本 */          
//    lv_label_set_text(label, "#0000ff Re-color# #ff00ff words# #ff0000 of a# "  
//                             "label, align the lines to the center" 
//                             "and  wrap long text automatically."); 
// /* 启用标签文本重新着色 */ 
//lv_label_set_recolor(label, true);       
///* 设置标签文本字体 */  
//lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);  
///* 设置标签宽度 */ 
//lv_obj_set_width(label, scr_act_width() / 3);           
///* 设置标签位置 */ 
//lv_obj_align(label, LV_ALIGN_CENTER, -scr_act_width() / 3, 0);  
///* 设置标签文本对齐方式 */ 
//    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);   
//} 


///** 
// * @brief  例2 
// * @param  无 
// * @return 无 
// */ 
//static void lv_example_label_2(void) 
//{ 
// /* 定义并创建标签 */ 
//lv_obj_t* label = lv_label_create(lv_scr_act());   
///* 设置标签文本 */                               
//lv_label_set_text(label, "It is a circularly scrolling text. ");  
///* 设置标签文本字体 */ 
//lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);  
///* 设置标签宽度 */  
//lv_obj_set_width(label, scr_act_width() / 3);  
///* 设置标签长模式：循环滚动 */   
//lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);  
///* 设置标签位置 */ 
//    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);  
//} 
// 
///** 
// * @brief  例3 
// * @param  无 
// * @return 无 
// */ 
//static void lv_example_label_3(void) 
//{ 
// /* 定义并创建标签 */ 
//lv_obj_t* label = lv_label_create(lv_scr_act());  
///* 设置标签文本 */    
//lv_label_set_text_fmt(label, "Label can set text like %s", "printf");   
///* 设置标签文本字体 */ 
//lv_obj_set_style_text_font(label,&lv_font_montserrat_18,LV_PART_MAIN);  
///* 设置标签宽度 */ 
//lv_obj_set_width(label, scr_act_width() / 3);   
///* 设置标签位置 */ 
//lv_obj_align(label, LV_ALIGN_CENTER, scr_act_width() / 3, 0);  
///* 设置标签文本对齐方式 */ 
//    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);   
// 
// /* 定义并创建阴影标签 */ 
// 

//lv_obj_t* label_shadow = lv_label_create(lv_scr_act());    
///* 设置标签文本 */ 
//lv_label_set_text(label_shadow, lv_label_get_text(label));     
///* 设置标签文本字体 */ 
//lv_obj_set_style_text_font(label_shadow, &lv_font_montserrat_18, LV_PART_MAIN);   
///* 设置标签宽度 */  
//lv_obj_set_width(label_shadow, scr_act_width() / 3);    
///* 设置标签文本透明度 */ 
//lv_obj_set_style_text_opa(label_shadow, LV_OPA_30, LV_PART_MAIN);  
///* 设置标签文本颜色 */ 
//lv_obj_set_style_text_color(label_shadow, lv_color_black(), LV_PART_MAIN);  
///* 设置标签文本对齐方式 */ 
//lv_obj_set_style_text_align(label_shadow, LV_TEXT_ALIGN_CENTER,  
//LV_PART_MAIN);   
// /* 设置标签位置 */ 
//    lv_obj_align_to(label_shadow, label, LV_ALIGN_TOP_LEFT, 3, 3);  
//} 
