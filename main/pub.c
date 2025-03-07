#include "time.h"
#include "sys/time.h"
#include "sys/unistd.h"
#include "esp_log.h"

#include "esp_timer.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "pub.h"

#define TAG "PUB"

uint8_t DebugOn = 1;

long MainLoop = 0;
long CurrentTime = 0;

long UpdateCurrentTime(void)
{
    CurrentTime=esp_timer_get_time()/1000;
    return CurrentTime;
}


