#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"

#include "esp_timer.h"
#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"

#define DIG1 21
#define DIG2 22
#define DIG3 23

#define A 19
#define B 18
#define C 5
#define D 17
#define E 16
#define F 4
#define G 2

#define LED_RED 27
#define LED_GREEN 13

#define Left_button 26
#define right_button 14

#define Left 33
#define right 32

#define SAMPLE_PERIOD_US 20000

//Variables
static volatile int value1=0;
static volatile int value2=0;
static volatile int value3=0;

static volatile int direccion=0;
static int act=0;

static volatile int64_t antireizq=0;
static volatile int64_t antireder=0;

//Display 7seg
const uint8_t digitos[10]={
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111
};

void mostrar_segmentos(uint8_t patron){
    gpio_set_level(A,!((patron>>0)&1));
    gpio_set_level(B,!((patron>>1)&1));
    gpio_set_level(C,!((patron>>2)&1));
    gpio_set_level(D,!((patron>>3)&1));
    gpio_set_level(E,!((patron>>4)&1));
    gpio_set_level(F,!((patron>>5)&1));
    gpio_set_level(G,!((patron>>6)&1));
}

void multiplexar_display(int n1,int n2,int n3){
    gpio_set_level(DIG1,1);
    gpio_set_level(DIG2,1);
    gpio_set_level(DIG3,1);

    mostrar_segmentos(digitos[n1]);
    gpio_set_level(DIG1,0);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(DIG1,1);
    gpio_set_level(DIG2,1);
    gpio_set_level(DIG3,1);

    mostrar_segmentos(digitos[n2]);
    gpio_set_level(DIG2,0);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(DIG1,1);
    gpio_set_level(DIG2,1);
    gpio_set_level(DIG3,1);

    mostrar_segmentos(digitos[n3]);
    gpio_set_level(DIG3,0);
    vTaskDelay(pdMS_TO_TICKS(5));
}

//Interrupciones

static void IRAM_ATTR izquierda(void *arg){
    int64_t now=esp_timer_get_time();

    if(now-antireizq>200000)
    {
        direccion=1;
        antireizq=now;
    }
}

static void IRAM_ATTR derecha(void *arg){
    int64_t now=esp_timer_get_time();

    if(now-antireder>200000)    {
        direccion=2;
        antireder=now;
    }
}

//Programa princpioal
void app_main(void){

// GPIO salida

gpio_config_t leds={
    .pin_bit_mask=
    (1ULL<<A)|
    (1ULL<<B)|
    (1ULL<<C)|
    (1ULL<<D)|
    (1ULL<<E)|
    (1ULL<<F)|
    (1ULL<<G)|
    (1ULL<<DIG1)|
    (1ULL<<DIG2)|
    (1ULL<<DIG3)|
    (1ULL<<LED_RED)|
    (1ULL<<LED_GREEN)|
    (1ULL<<Left)|
    (1ULL<<right),

    .mode=GPIO_MODE_OUTPUT
};

ESP_ERROR_CHECK(gpio_config(&leds));

// GPIO entradas

gpio_config_t inputs={
    .pin_bit_mask=
    (1ULL<<Left_button)|
    (1ULL<<right_button),

    .mode=GPIO_MODE_INPUT,
    .pull_up_en=GPIO_PULLUP_ENABLE,
    .intr_type=GPIO_INTR_NEGEDGE
};

ESP_ERROR_CHECK(gpio_config(&inputs));

gpio_install_isr_service(0);

gpio_isr_handler_add(Left_button,izquierda,NULL);
gpio_isr_handler_add(right_button,derecha,NULL);

//Coonfig Timers (Versión 6.0 de ESP-IDF)
gptimer_handle_t adc_timer=NULL;

gptimer_config_t timer_cfg={
    .clk_src=GPTIMER_CLK_SRC_DEFAULT,
    .direction=GPTIMER_COUNT_UP,
    .resolution_hz=1000000
};

ESP_ERROR_CHECK(
gptimer_new_timer(&timer_cfg,&adc_timer)
);

ESP_ERROR_CHECK(
gptimer_enable(adc_timer)
);

ESP_ERROR_CHECK(
gptimer_set_raw_count(adc_timer,0)
);

ESP_ERROR_CHECK(
gptimer_start(adc_timer)
);

uint64_t timer_value=0;

//Lectura de datos potenciometro
adc_oneshot_unit_handle_t adc1_handle;

adc_oneshot_unit_init_cfg_t init_config={
.unit_id=ADC_UNIT_1
};

ESP_ERROR_CHECK(
adc_oneshot_new_unit(
&init_config,
&adc1_handle
)
);

adc_oneshot_chan_cfg_t config={
.atten=ADC_ATTEN_DB_12,
.bitwidth=ADC_BITWIDTH_DEFAULT
};

ESP_ERROR_CHECK(
adc_oneshot_config_channel(
adc1_handle,
ADC_CHANNEL_6,
&config
)
);

int adc_raw=0;

//Pwm
ledc_timer_config_t ledc_timer={
.speed_mode=LEDC_LOW_SPEED_MODE,
.timer_num=LEDC_TIMER_0,
.duty_resolution=LEDC_TIMER_12_BIT,
.freq_hz=5000,
.clk_cfg=LEDC_AUTO_CLK
};

ledc_timer_config(&ledc_timer);

ledc_channel_config_t channel={
.gpio_num=25,
.speed_mode=LEDC_LOW_SPEED_MODE,
.channel=LEDC_CHANNEL_0,
.timer_sel=LEDC_TIMER_0,
.duty=0
};

ledc_channel_config(&channel);

while(1)
{

multiplexar_display(
value1,
value2,
value3
);

// reemplazo timer_get_counter_value()

gptimer_get_raw_count(
adc_timer,
&timer_value
);

if(timer_value>=SAMPLE_PERIOD_US)
{
adc_oneshot_read(
adc1_handle,
ADC_CHANNEL_6,
&adc_raw
);

int porcentaje=
100-
((adc_raw*100)/4095);

value1=porcentaje/100;
value2=(porcentaje/10)%10;
value3=porcentaje%10;

// reemplazo timer_set_counter_value()

gptimer_set_raw_count(
adc_timer,
0
);

int duty=adc_raw;

ledc_set_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0,
duty
);

ledc_update_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0
);
}

// lógica dirección

if(direccion==1 && act==0)
{
ledc_set_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0,
4095
);

ledc_update_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0
);

vTaskDelay(pdMS_TO_TICKS(2000));

gpio_set_level(LED_RED,1);
gpio_set_level(LED_GREEN,0);

gpio_set_level(Left,1);
gpio_set_level(right,0);

direccion=0;
act=1;
}

else if(direccion==2 && act==1)
{
ledc_set_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0,
4095
);

ledc_update_duty(
LEDC_LOW_SPEED_MODE,
LEDC_CHANNEL_0
);

vTaskDelay(pdMS_TO_TICKS(2000));

gpio_set_level(LED_RED,0);
gpio_set_level(LED_GREEN,1);

gpio_set_level(Left,0);
gpio_set_level(right,1);

direccion=0;
act=0;
}

}

}