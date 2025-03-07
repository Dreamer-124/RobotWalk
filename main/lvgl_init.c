#include "lvgl_init.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/

void lv_tick_task(void *arg)
{
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

/*********************** GUI_SHOW_CODE_START***********************/
lv_obj_t * label_1;
lv_obj_t * label_2;
lv_obj_t * label_3;


extern lv_font_t myFontCN;
extern lv_font_t myFontCN32;
extern lv_img_dsc_t jf_logo_w_30_40;
extern lv_img_dsc_t jf_logo_w_90_120;
void Show_State()
{

    static lv_style_t bg_style;
    lv_style_init(&bg_style);
    //lv_style_set_text_color(&bg_style, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    //lv_style_set_text_font(&label_style, LV_STATE_DEFAULT, &lv_font_montserrat_24);
    lv_style_set_text_font(&label_style, LV_STATE_DEFAULT, &myFontCN);

    static lv_style_t label_style32;
    lv_style_init(&label_style32);
    lv_style_set_text_color(&label_style32, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_style_set_text_font(&label_style32, LV_STATE_DEFAULT, &myFontCN32);

    lv_obj_t *scr = lv_scr_act(); //创建scr
    lv_obj_set_pos(scr,0,0);
    lv_scr_load(scr);


    // lv_obj_t * img1 = lv_img_create(lv_scr_act(), NULL);
    // lv_img_set_src(img1, &jf_logo_w_90_120);
    // //lv_obj_align(img2, NULL, LV_ALIGN_CENTER, 0, -20);
    // lv_obj_set_pos(img1,75,110);//设置位置

    lv_obj_t * img2 = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(img2, &jf_logo_w_30_40);
    //lv_obj_align(img2, NULL, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_pos(img2,10,10);//设置位置

    lv_obj_t* bgk;

    bgk = lv_scr_act();//lv_obj_create(scr, NULL);//创建对象
    lv_obj_add_style(bgk, LV_OBJ_PART_MAIN, &bg_style);

    //省去下方两行代码，默认是从0,0处开始绘制
	//lv_obj_set_x(bgk, 0);//设置X轴起点
	//lv_obj_set_y(bgk, 0);//设置Y轴起点
	//lv_obj_set_size(bgk, 240, 120);//设置覆盖大小

    label_1 =lv_label_create(bgk,0);//创建label
    lv_label_set_recolor(label_1,1);//颜色可变换
    lv_label_set_long_mode(label_1,LV_LABEL_LONG_SROLL);//设置滚动模式
    lv_obj_set_pos(label_1,50,13);//设置位置
    lv_obj_set_size(label_1,220,64);//设定大小
    lv_obj_add_style(label_1, LV_LABEL_PART_MAIN, &label_style32);
    lv_label_set_text(label_1, "\uF061测试文字#FF0000 \uF004#");//设定文本内容


    label_2 =lv_label_create(bgk,0);//创建label
    lv_label_set_recolor(label_2,1);//颜色可变换
    lv_label_set_long_mode(label_2,LV_LABEL_LONG_SROLL);//设置滚动模式
    lv_obj_set_pos(label_2,10,60);//设置位置
    lv_obj_set_size(label_2,220,200);//设定大小
    lv_obj_add_style(label_2, LV_LABEL_PART_MAIN, &label_style);
    lv_label_set_text(label_2, "This is the Intetnet thread aaaaa");//设定文本内容


    label_3 =lv_label_create(bgk,0);//创建label
    lv_label_set_recolor(label_3,1);//颜色可变换
    lv_label_set_long_mode(label_3,LV_LABEL_LONG_SROLL);//设置滚动模式
    lv_obj_set_pos(label_3,10,210);//设置位置
    lv_obj_set_size(label_3,220,100);//设定大小
    lv_obj_add_style(label_3, LV_LABEL_PART_MAIN, &label_style);
    lv_label_set_text(label_3, "============");//设定文本内容



}
/*********************** GUI_SHOW_CODE_END***********************/

void guiTask1(void *pvParameter)
{

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
   // memset(buf1,0x00ff,DISP_BUF_SIZE * sizeof(lv_color_t));
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    //memset(buf2,0x00ff,DISP_BUF_SIZE * sizeof(lv_color_t));
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res=LV_HOR_RES_MAX;
    disp_drv.ver_res=LV_VER_RES_MAX;
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);
    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    Show_State();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }
    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}
