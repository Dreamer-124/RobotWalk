#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lvgl.h"
#include "freertos/semphr.h"
#include "esp_system.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"
#include "src\lv_hal\lv_hal_disp.h"

#define LV_TICK_PERIOD_MS 1
#define BUF_W 20
#define BUF_H 10

// 声明全局变量,可被所有文件使用
extern SemaphoreHandle_t xGuiSemaphore;
extern lv_obj_t * label_1;
extern lv_obj_t * label_2;
extern lv_obj_t * label_3;

// 声明函数
void lv_tick_task(void *arg);
void guiTask1(void *pvParameter);
void Show_State();
