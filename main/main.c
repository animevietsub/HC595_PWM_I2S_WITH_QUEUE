/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : HC595_PWM_I2S_WITH_QUEUE
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 Espressif.
 * All rights reserved.
 *
 * Vo Duc Toan / B1907202
 * Can Tho University.
 * March - 2022
 * Built with ESP-IDF Version: 4.4.
 * Target device: ESP32-WROOM.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_log.h"

#define PWM_FREQUENCY (16 * 1000)
#define PWM_TIMER_GROUP (TIMER_GROUP_0) // Timer group
#define PWM_TIMER_IDX (TIMER_0)         // Timer index
#define PWM_TIMER_DIVIDER (16)          //  Hardware timer clock divider
#define PWM_SCALE (TIMER_BASE_CLK / PWM_TIMER_DIVIDER)
#define PWM_ALARM_VALUE (PWM_SCALE / PWM_FREQUENCY)

#define HC595_CLKFREQ (8 * 1000 * 1000)
#define I2S_NUM_CHANNEL 2
#define I2S_NUM_BIT I2S_BITS_PER_SAMPLE_16BIT
#define I2S_NUM (0)
#define I2S_WS_PERIOD ((16 * I2S_NUM_CHANNEL) / (HC595_CLKFREQ / (1000 * 1000))) // 16 bit data - 4 us
#define DMA_BUFFER_LENGTH 64
#define DMA_BUFFER_COUNT 16
#define DMA_BUFFER_PREPARE (DMA_BUFFER_LENGTH * 2) // 64 queues * 2 channels
#define QUEUE_DMA_MULTIPLIER 8

#define HC595_NUM_RCLK 21
#define HC595_NUM_SRCLK 22
#define HC595_NUM_SER 23

#define PWM_MOTOR_FREQUENCY (800)
#define PWM_SERVO_FREQUENCY (50)

#define MLA_PINOUT (BIT0)
#define MLB_PINOUT (BIT1)
#define PWM_ML_PINOUT (BIT2)
#define MRA_PINOUT (BIT3)
#define MRB_PINOUT (BIT4)
#define PWM_MR_PINOUT (BIT5)
#define PWM_SERVO_PINOUT (BIT6)

DRAM_ATTR uint16_t HC595_BUFFER = 0x0000;
DRAM_ATTR uint16_t TIMER_ML_COUNTER = 0;
DRAM_ATTR uint16_t TIMER_MR_COUNTER = 0;
DRAM_ATTR uint16_t TIMER_SERVO_COUNTER = 0;

DRAM_ATTR QueueHandle_t xQueue1;
DRAM_ATTR int64_t timeStartReceive, timeEndReceive;

DRAM_ATTR uint8_t D_PWM_ML = 20;
DRAM_ATTR uint8_t D_PWM_MR = 10;
DRAM_ATTR uint8_t D_PWM_SERVO = 2;

static const char *TAG = "[PWM_TIMER]";

IRAM_ATTR void HC595_QueueDelayI2S(uint32_t delayUs)
{
    uint16_t HC595_TEMP_BUFFER = HC595_BUFFER;
    for (uint32_t i = 0; i < delayUs / I2S_WS_PERIOD; i++)
    {
        xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
    }
}

static void HC595_TaskSend()
{
    size_t i2s_bytes_write = DMA_BUFFER_PREPARE * sizeof(uint16_t);                                      // For first writing
    uint16_t *sampleData = heap_caps_malloc(DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1, MALLOC_CAP_8BIT); // Create DMA-buffer
    memset(sampleData, 0x0000, DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1);                               // Clear memory data
    uint16_t *sampleDataBegin = sampleData;                                                              // sampleData begin address
    uint16_t lastData = 0x0000;
    uint32_t queueMessagesWaiting = 0;
    // uint8_t dmaSelect = 0;
    while (1)
    {
        // if (dmaSelect == 0) // Change dma buffer
        // {
        // 	sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
        // 	dmaSelect = 1;
        // }
        // else if (dmaSelect == 1)
        // {
        // 	sampleData = sampleDataBegin + DMA_BUFFER_PREPARE + 1; // Start from second-half, add 1 more shift
        // 	dmaSelect = 0;
        // }
        sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
        // timeStartReceive = esp_timer_get_time();
        for (uint16_t i = 0; i < DMA_BUFFER_PREPARE / 2; i++)
        {
            if (uxQueueMessagesWaiting(xQueue1) > 0)
            {
                queueMessagesWaiting = uxQueueMessagesWaiting(xQueue1);
                if ((DMA_BUFFER_PREPARE / 2) - i < queueMessagesWaiting)
                {
                    queueMessagesWaiting = (DMA_BUFFER_PREPARE / 2) - i;
                }
                for (uint32_t temp = 0; temp < queueMessagesWaiting; temp++)
                {
                    xQueueReceive(xQueue1, sampleData, portMAX_DELAY); // Get new data
                    lastData = *sampleData;
                    sampleData++;
                    *sampleData = 0x0000;
                    sampleData++;
                }
                i += queueMessagesWaiting - 1;
            }
            else
            {
                *sampleData = lastData;
                sampleData++;
                *sampleData = 0x0000;
                sampleData++;
            }
        }
        // timeEndReceive = esp_timer_get_time();
        // ESP_LOGI("[Queue]", "%lld us", timeEndReceive - timeStartReceive);
        sampleData = sampleData - DMA_BUFFER_PREPARE - 1; // Remove the shift
        // timeStartReceive = esp_timer_get_time();
        i2s_write(I2S_NUM, sampleData, DMA_BUFFER_PREPARE * sizeof(uint16_t), &i2s_bytes_write, portMAX_DELAY);
        // timeEndReceive = esp_timer_get_time();
        // ESP_LOGI("[I2S]", "%lld us", timeEndReceive - timeStartReceive);
    }
    heap_caps_free(sampleData);
    vTaskDelete(NULL);
}

void HC595_I2SInit()
{
    xQueue1 = xQueueCreate(DMA_BUFFER_PREPARE * QUEUE_DMA_MULTIPLIER, sizeof(uint16_t));
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = (HC595_CLKFREQ / I2S_NUM_CHANNEL / I2S_NUM_BIT),
        .bits_per_sample = I2S_NUM_BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LENGTH,
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = HC595_NUM_SRCLK,
        .ws_io_num = HC595_NUM_RCLK,
        .data_out_num = HC595_NUM_SER,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    // SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A_V, 1, I2S_CLKM_DIV_A_S);
    // SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B_V, 1, I2S_CLKM_DIV_B_S);
    // SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM_V, 2, I2S_CLKM_DIV_NUM_S);
    // SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BCK_DIV_NUM_V, 2, I2S_TX_BCK_DIV_NUM_S);
    i2s_start(I2S_NUM);
    xTaskCreatePinnedToCore(HC595_TaskSend, "[HC595_TaskSend]", 1024 * 4, NULL, 3, NULL, 1);
}

IRAM_ATTR void HC595_SendDataToQueue() // Add new data to Queue
{
    uint16_t HC595_TEMP_BUFFER = HC595_BUFFER;
    xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
}

IRAM_ATTR void PWM_SetDutyCycle(uint16_t *COUNTER, uint16_t FREQUENCY, uint16_t PIN_OUT, uint8_t DUTY_CYCLE)
{
    if (*COUNTER < (PWM_FREQUENCY / FREQUENCY) * DUTY_CYCLE / 100)
    {
        HC595_BUFFER |= PIN_OUT;
    }
    else
    {
        HC595_BUFFER &= ~PIN_OUT;
    }
}

IRAM_ATTR void PWM_CheckDutyCycle(uint16_t *COUNTER, uint16_t FREQUENCY)
{
    *COUNTER = *COUNTER + 1;
    if (*COUNTER >= (PWM_FREQUENCY / FREQUENCY))
    {
        *COUNTER = 0;
    }
}

static bool IRAM_ATTR pwm_isr_callback() // PWM ISR send to I2S
{
    PWM_SetDutyCycle(&TIMER_ML_COUNTER, PWM_MOTOR_FREQUENCY, PWM_ML_PINOUT, D_PWM_ML);
    PWM_SetDutyCycle(&TIMER_MR_COUNTER, PWM_MOTOR_FREQUENCY, PWM_MR_PINOUT, D_PWM_MR);
    PWM_SetDutyCycle(&TIMER_SERVO_COUNTER, PWM_SERVO_FREQUENCY, PWM_SERVO_PINOUT, D_PWM_SERVO);
    HC595_QueueDelayI2S(1000000 / PWM_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_ML_COUNTER, PWM_MOTOR_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_MR_COUNTER, PWM_MOTOR_FREQUENCY);
    PWM_CheckDutyCycle(&TIMER_SERVO_COUNTER, PWM_SERVO_FREQUENCY);
    return pdTRUE;
}

static void testPWM()
{
    while (1)
    {
        if (D_PWM_SERVO < 12)
        {
            D_PWM_SERVO += 1;
        }
        else
        {
            D_PWM_SERVO = 2;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    HC595_I2SInit();
    xTaskCreate(testPWM, "[testPWM]", 1024 * 3, NULL, 2, NULL);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    timer_config_t config = {
        .divider = PWM_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(PWM_TIMER_GROUP, PWM_TIMER_IDX, &config);
    timer_set_counter_value(PWM_TIMER_GROUP, PWM_TIMER_IDX, 0);
    timer_set_alarm_value(PWM_TIMER_GROUP, PWM_TIMER_IDX, PWM_ALARM_VALUE);
    timer_enable_intr(PWM_TIMER_GROUP, PWM_TIMER_IDX);
    timer_isr_callback_add(PWM_TIMER_GROUP, PWM_TIMER_IDX, pwm_isr_callback, NULL, 0);
    timer_start(PWM_TIMER_GROUP, PWM_TIMER_IDX);
    ESP_LOGI(TAG, "%d", PWM_ALARM_VALUE);
}
