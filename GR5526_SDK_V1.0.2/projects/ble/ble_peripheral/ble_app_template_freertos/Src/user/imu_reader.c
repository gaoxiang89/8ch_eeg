#include "ads1299/ads1299.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "app_log.h"
#include <app_io.h>
#include "qmi8658.h"
#include "qmc6309.h"

// Declare the semaphore handle.
static SemaphoreHandle_t xSemaphore = NULL;

// This is the timer callback function.
void vTimerCallback(TimerHandle_t xTimer)
{
    // Give the semaphore.
    xSemaphoreGive(xSemaphore);
}

static float acc[3], gyro[3], mag[3];

void imu_reader_task(void *arg)
{
    xSemaphore = xSemaphoreCreateBinary();

    // Create the timer.
    TimerHandle_t xTimer = xTimerCreate("ReadTimer", pdMS_TO_TICKS(1000), pdTRUE, 0, vTimerCallback);

    // Start the timer.
    xTimerStart(xTimer, 0);

    TickType_t xLastReadTime = xTaskGetTickCount();
    TickType_t xCurrentReadTime;

    qmc6309_init();
    qmi8658_init();

    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            qmi8658_read_xyz(acc, gyro);
            qmc6309_read_mag_xyz(mag);

            // Get the current read time
            xCurrentReadTime = xTaskGetTickCount();

            // Calculate the time difference in ms
            TickType_t xTimeDifference = (xCurrentReadTime - xLastReadTime);

            // APP_LOG_INFO("Time difference between two readings: %d ms\n", xTimeDifference);

            // Update the last read time
            xLastReadTime = xCurrentReadTime;
        }
    }
}

void imu_reader_init(void)
{
    xTaskCreate(imu_reader_task, "imu_reader_task", 1024, NULL, 2, NULL);
}