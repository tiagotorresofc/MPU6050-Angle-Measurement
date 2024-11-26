#include <cstdio>
#include <cstring>  // Para usar memset
#include "mpu6050.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

// Instance of the MPU6050 sensor
mpu6050 mpu6050(i2c0, 4, 5);

// Define a buffer to hold the printed data
#define PRINT_BUFFER_SIZE 512
char printBuffer[PRINT_BUFFER_SIZE];

/**
 * [NAME]:        mpu_task
 * [FUNCTION]:    mpu_task(void *pvParameters)
 * [PARAMETERS]:  void *pvParameters - FreeRTOS parameter for the task (not used)
 * [DESCRIPTION]: This function reads the accelerometer data and calculates the angles
 *                using the methods from the mpu6050 class. It prints the raw angle (Old)
 *                and filtered angle in a simple format every second.
 */
void mpu_task(void *pvParameters)
{
    while (1)
    {
        float angle, angleFiltered;
        
        // Get the angle values from different methods
        mpu6050.getAngle(angle);               // Get raw angle (old method, no filtering)
        mpu6050.getAngleFiltered(angleFiltered); // Get filtered angle with EMA

        // Clear the print buffer before adding new data
        memset(printBuffer, 0, PRINT_BUFFER_SIZE);

        // Add the formatted data to the print buffer (simplified version)
        snprintf(printBuffer, PRINT_BUFFER_SIZE, 
                 "Raw Angle: %-6.2f | Filtered Angle: %-6.2f\n", 
                 angle, angleFiltered);

        // Print the contents of the buffer
        printf("%s", printBuffer);

        // Delay for 1 second before updating the angles
        vTaskDelay(700 / portTICK_PERIOD_MS);
    }
}

/**
 * [NAME]:        main
 * [FUNCTION]:    main()
 * [PARAMETERS]:  void
 * [DESCRIPTION]: The main function initializes the standard I/O, creates the FreeRTOS task
 *                to handle the sensor readings, and starts the FreeRTOS scheduler.
 */
int main()
{
    // Initialize standard I/O
    stdio_init_all();

    // Create the FreeRTOS task for the MPU sensor
    xTaskCreate(mpu_task, "MPU Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Infinite loop, execution never reaches here
    while (true);
}

