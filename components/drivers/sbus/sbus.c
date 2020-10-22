#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "sbus.h"
#include "log.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "driver/gpio.h"

#define DEBUG_MODULE "SBUS"
#include "debug_cf.h"

static const char *TAG = "uart_events";

#define EX_UART_NUM UART_NUM_2
#define PATTERN_CHR_NUM    (20)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (128)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

uint16_t rc_data[16] = {1000, 1000, 1500, 1000};
uint8_t dtmp[RD_BUF_SIZE];
float rch, pch, ych;
static uint16_t tch,is_send=0;
static int sbus_parse(uint16_t *rc_data,uint8_t *buf)
{
		// if (buf[0] != 0x0F)
		// 	return 1;

		// if (buf[24] != 0x00)
		// 	return 1;

		rc_data[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
		rc_data[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
		rc_data[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
		rc_data[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);

		rc_data[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
		rc_data[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
		rc_data[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
		rc_data[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
		// rc_data[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
		// rc_data[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
		// rc_data[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
		// rc_data[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
		// rc_data[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
		// rc_data[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
		// rc_data[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
		// rc_data[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);

        return 0;
}

// void printf_byte_logd(uint8_t *buf,uint16_t len)
// {
// 	uint8_t buffer_tmp[4];
// 	uint8_t buffer_log[128];	
// 	uint16_t count = 0;
// 	buffer_log[0] = '\0';

// 	if(len > 1024)
// 		return;

// 	for(;count < len;count++)
// 	{
		
// 		sprintf((char *)buffer_tmp, "%02X ",*(buf + count));
// 		strcat((char *)buffer_log,(const char *)buffer_tmp);
// 	}
// 	DEBUG_PRINT("%s",buffer_log);
// }
// float get_roll(void)
// {

// }

uint16_t get_tch(void)
{
    return tch;
}

float get_rch(void)
{
    return rch;
}
float get_pch(void)
{
    return -pch;
}
float get_ych(void)
{
    return ych;
}
uint16_t get_is_send(void)
{
    return is_send;
}
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {

            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, (uint8_t *)dtmp, event.size, portMAX_DELAY);
                    if(event.size == 25)
                    {
                        //printf_byte_logd(dtmp,3);
                        sbus_parse(rc_data,dtmp+1);
                        //rc_data[0] = ((dtmp[1] | dtmp[2] << 8) & 0x07FF);
                        //DEBUG_PRINT("%02X\t%02X\t%02X\t%02X\t%d",dtmp[0],dtmp[1],dtmp[2],dtmp[3],rc_data[0]);
                        
                        
                        rch  = (float)(((float)rc_data[3] - 1024)/1024 * 15);
                        pch  = (float)(((float)rc_data[1] - 1024)/1024 * 15);
                        ych  = (float)(((float)rc_data[0] - 1024)/1024 * 90);

                        if(rch < 1.0f && rch > -1.0f)
                            rch = 0;
                        if(pch < 1.0f && pch > -1.0f)
                            pch = 0;
                        if(ych < 2.0f && ych > -2.0f)
                            ych = 0;

                        if(rc_data[2] < 300)
                            rc_data[2] = 200;
                        if(rc_data[2] > 2000)
                            rc_data[2] = 2000;                            
                        tch  = (rc_data[2] - 200) * 30;  //1000-60000
                        
                        if(rc_data[5] > 1800)
                        {
                            is_send=1;
                        }else{
                            is_send=0;
                            //DEBUG_PRINT("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",rc_data[0],rc_data[1],rc_data[2],rc_data[3],rc_data[4],rc_data[5],rc_data[6],rc_data[7]);
                        }
                        
                    }
                    // ESP_LOGI(TAG, "[DATA EVT]:");
                    // uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    //ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    //ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    //ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    //ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    //ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    //ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        //ESP_LOGI(TAG, "read data: %s", dtmp);
                        //ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    //ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

void sbus_init()
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 40, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, GPIO_NUM_27, GPIO_NUM_26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 40);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

}