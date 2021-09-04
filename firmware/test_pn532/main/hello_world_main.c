/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "PN532_I2C.h"

#define TAG "MAIN"
#define CHIP_NAME "ESP32"

#define PN532_I2C_SDA 5
#define PN532_I2C_SCL 18
#define PN532_IRQ 23

void app_main(void)
{
    uint8_t success;    
    uint8_t i;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    

    printf("Hello world!\n");
    init_PN532_I2C(PN532_I2C_SDA, PN532_I2C_SCL, PN532_IRQ, I2C_NUM_0);
    SAMConfig();
    getPN532FirmwareVersion();
    while(true) 
    {
        success = readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);    
        if (success) {
            // Display some basic information about the card
            printf("Found an ISO14443A card\n");
            printf("  UID Length: %d\n", uidLength);
            printf("  UID Value: ");
            for (i=0;i<uidLength;i++) {
                printf("%d", uid[i]);
                if(i<(uidLength-1)) {
                    printf(".");
                }
            }
            printf("\n");
        } else {
            printf("No Card found\n");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
