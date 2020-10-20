#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lora.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

/* Private definitions and enumerations */
#define SLAVERX_MASTERTX_MSG_SIZE	2
#define UART_RX_BUFFER_SIZE			256
#define SLAVE_INPUT_0				GPIO_NUM_12
#define SLAVE_INPUT_1				GPIO_NUM_14
#define SLAVE_OUTPUT_2				GPIO_NUM_2
#define SLAVE_OUTPUT_3				GPIO_NUM_4

typedef enum {
	SLAVE_ST_IDLE_WAIT_MASTERTX = 0,
	SLAVE_ST_CHECK_SLAVE_ADDR,
	SLAVE_ST_PARSE_MASTER_MSG,
	SLAVE_ST_SLAVE_RESPONSE,
} slave_state_t;

typedef struct {
	bool inputs[4];
	bool outputs[4];
} slave_ctrl_t;

/* Private variables */
uint8_t slavetx_msg;
uint8_t slaverx_msg[SLAVERX_MASTERTX_MSG_SIZE];

slave_state_t slave_state = SLAVE_ST_IDLE_WAIT_MASTERTX;
slave_ctrl_t slave_ctrl = {0};
bool new_message_arrived = false;
uint8_t current_slave_address = 0; // TODO Use static RAM attribute

/* Private functions */
void lora_rx_done_callback(uint8_t* buffer_rx, int pac_size)
{
	/* Slave must receive packets only from the Master */
	if ( pac_size == 2 ){
		memset(&slaverx_msg, 0, sizeof(slaverx_msg));
		memcpy(&slaverx_msg, buffer_rx, sizeof(slaverx_msg));
		new_message_arrived = true;
	}
	lora_receive_from_isr();
}

void parse_slave_response_message (uint8_t* message)
{
	*message = 0;

	uint8_t start_bit = 0;
	uint8_t data_bits = 0;

	slave_ctrl.inputs[0] = gpio_get_level(SLAVE_INPUT_0);
	slave_ctrl.inputs[1] = gpio_get_level(SLAVE_INPUT_1);
	slave_ctrl.inputs[2] = 0;
	slave_ctrl.inputs[3] = 0;

	data_bits |= (((uint8_t)(slave_ctrl.inputs[3])) & 0x01) << 1;
	data_bits |= (((uint8_t)(slave_ctrl.inputs[2])) & 0x01) << 2;
	data_bits |= (((uint8_t)(slave_ctrl.inputs[1])) & 0x01) << 3;
	data_bits |= (((uint8_t)(slave_ctrl.inputs[0])) & 0x01) << 4;

	*message |= start_bit;
	*message |= data_bits;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 5; i++) {
		number_of_ones += (*message >> i) & 0x01;
	}
	uint8_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint8_t end_bit = 1;

	*message |= parity_bit << 5;
	*message |= end_bit << 6;

#ifdef DEBUG
	printf("Sent response msg: %02X\n", (unsigned int)(*message));
#endif
}

uint8_t get_slave_address_from_master_message (uint8_t* message)
{
	uint16_t raw_message = 0;

	raw_message |= ( (uint16_t)(message[0]) );
	raw_message |= ( ( (uint16_t)(message[1]) ) << 8 );

	uint8_t address_bits = (uint8_t)( (raw_message >> 2) & 0x001F );

	return address_bits;
}

void parse_received_message_from_master (uint8_t* message)
{
	uint8_t number_of_ones = 0;
	uint16_t raw_message = 0;

	raw_message |= ( (uint16_t)(message[0]) );
	raw_message |= ( ( (uint16_t)(message[1]) ) << 8 );

	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x01;
	}

	uint8_t parity_bit_calculated = number_of_ones % 2;
	uint8_t parity_bit_from_msg = (raw_message >> 12) & 0x01;

	/* Parity bit for message integrity checking */
	if (parity_bit_from_msg == parity_bit_calculated) {
		uint8_t command_bit = (uint8_t)( (raw_message >> 1) & 0x0001 );
		uint8_t address_bits = (uint8_t)( (raw_message >> 2) & 0x001F );
		uint8_t data_bits = (uint8_t)( (raw_message >> 7) & 0x001F );

#ifdef DEBUG
		printf("Received master msg: %04X\n", (unsigned int)(raw_message));
#endif

		if ( address_bits != 0 ) {
			if ( command_bit == 0 ) {
				if ( (data_bits & 0x01) == 0 ) {
					/* Data exchange message */
					if ( slave_ctrl.outputs[3] != ((bool)((data_bits >> 1) & 0x01)) ) {
						slave_ctrl.outputs[3] = (bool)((data_bits >> 1) & 0x01);
						gpio_set_level(SLAVE_OUTPUT_3, slave_ctrl.outputs[3]);
					}
					if ( slave_ctrl.outputs[2] != ((bool)((data_bits >> 2) & 0x01)) ) {
						slave_ctrl.outputs[2] = (bool)((data_bits >> 2) & 0x01);
						gpio_set_level(SLAVE_OUTPUT_2, slave_ctrl.outputs[2]);
					}
					if ( slave_ctrl.outputs[1] != ((bool)((data_bits >> 3) & 0x01)) ) {
						slave_ctrl.outputs[1] = (bool)((data_bits >> 3) & 0x01);
					}
					if ( slave_ctrl.outputs[0] != ((bool)((data_bits >> 4) & 0x01)) ) {
						slave_ctrl.outputs[0] = (bool)((data_bits >> 4) & 0x01);
					}
				}
				else {
					/* Parameters write message */
					/* Future implementation */
				}
			}
			else {
				if ( data_bits == 0 ) {
					/* Erase address command received */
					current_slave_address = 0;
				}
				else {
					/* Slave reset, I/O config, ID code, Memory Status, etc */
				}
			}
		}
		else {
			if ( command_bit == 0) {
				/* Set new address command received */
				current_slave_address = data_bits;
			}
		}
	}
}

void uart_interface_task(void *p)
{
    uint8_t data[ UART_RX_BUFFER_SIZE + 1 ] = {0};
    int uart_rxBytes = 0;
    uint32_t last_slave_info_print_tick = xTaskGetTickCount();

    static const char *uart_interface_task_tag = "uart_interface_task";
    esp_log_level_set(uart_interface_task_tag, ESP_LOG_INFO);

    /* There are two possible commands receivable:
     * <> Erase Slave Address	->	ERSADDRxx
     * <> Set New Slave Address	->	SETADDRxx
     * Where xx is the address of the slave (1 - 31)
     * */
    for (;;) {

    	/* UART command parser mechanism */
        uart_rxBytes = uart_read_bytes(UART_NUM_0, data, UART_RX_BUFFER_SIZE, 10 / portTICK_RATE_MS);
        if (uart_rxBytes > 0) {
            data[uart_rxBytes] = 0;

            if ( strncmp( "ERSADDR" , (char*)data , 7 ) == 0 ) {
            	if ( current_slave_address != 0 ) {
            		ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Erased slave address");
            		current_slave_address = 0;
            	}
            	else {
                    ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Address already erased");
            	}
            }
            else if ( strncmp( "SETADDR" , (char*)data , 7 ) == 0 ) {
            	int new_address_to_set = atoi( (char*)(data + 7) );
            	if ( (new_address_to_set > 0) && (new_address_to_set < 32) ) {
            		if ( current_slave_address == 0 ) {
            			/* Check if the slave 0 is available in the network */
            			current_slave_address = (uint8_t)new_address_to_set;
            			ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address %i assigned", new_address_to_set);
            		}
            		else {
                        ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address must be erased before setting a new");
            		}
            	}
            	else {
                    ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address invalid");
            	}
            }

            memset(data, 0, sizeof(data));
        }

        /* UART slaves info printer mechanism */
		if (xTaskGetTickCount() - last_slave_info_print_tick >= pdMS_TO_TICKS(2000)) {
			last_slave_info_print_tick = xTaskGetTickCount();
			printf(LOG_COLOR(LOG_COLOR_RED));
			printf("\n");
			printf("##########################################\n");
			printf("#            SLAVE INFO PRINT            #\n");
			printf("##########################################\n");
			printf("#ADDRESS #     INPUTS    #    OUTPUTS    #\n");
			printf("#        # 0 # 1 # 2 # 3 # 0 # 1 # 2 # 3 #\n");
			printf("##########################################\n");
			printf("#SLV %.2u  # %u # %u # %u # %u # %u # %u # %u # %u #\n",
					(unsigned int)current_slave_address,
					(unsigned int)slave_ctrl.inputs[0],
					(unsigned int)slave_ctrl.inputs[1],
					(unsigned int)slave_ctrl.inputs[2],
					(unsigned int)slave_ctrl.inputs[3],
					(unsigned int)slave_ctrl.outputs[0],
					(unsigned int)slave_ctrl.outputs[1],
					(unsigned int)slave_ctrl.outputs[2],
					(unsigned int)slave_ctrl.outputs[3]);
			printf("##########################################\n");
			printf(LOG_RESET_COLOR);
		}

		vTaskDelay(1);
    }
}

void slave_task(void *p)
{
	for(;;) {
		switch (slave_state) {
			case SLAVE_ST_IDLE_WAIT_MASTERTX:
				/* If arrived new message, get out of the idle state and
				 * check if is addressed to this slave */
				if (new_message_arrived) {
					new_message_arrived = false;
					slave_state = SLAVE_ST_CHECK_SLAVE_ADDR;
				}
				break;

			case SLAVE_ST_CHECK_SLAVE_ADDR:
				/* Verify if the message is addressed to THIS slave
				 * If true, parse the message and then goes to idle state
				 * Otherwise, goes directly to idle state */
				if ( get_slave_address_from_master_message(slaverx_msg) != current_slave_address ) {
					lora_receive();
					slave_state = SLAVE_ST_IDLE_WAIT_MASTERTX;
				}
				else {
					slave_state = SLAVE_ST_PARSE_MASTER_MSG;
				}
				break;

			case SLAVE_ST_PARSE_MASTER_MSG:
				parse_received_message_from_master(slaverx_msg);

				slave_state = SLAVE_ST_SLAVE_RESPONSE;
				break;

			case SLAVE_ST_SLAVE_RESPONSE:
				parse_slave_response_message(&slavetx_msg);
				lora_send_packet(&slavetx_msg, sizeof(slavetx_msg));
				lora_receive();

				slave_state = SLAVE_ST_IDLE_WAIT_MASTERTX;
				break;

			default:
				break;
		}

		vTaskDelay(1);
	}
}

void app_main()
{
	/* Non Volatile Storage initialization */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	/* LoRa initialization */
	lora_init();
	lora_set_frequency(915e6);
	lora_set_bandwidth(500e3);
	lora_set_spreading_factor(7);
	lora_enable_crc();
	lora_onReceive(&lora_rx_done_callback);
	lora_receive();

	/* UART interface initialization */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0);

    /* Inputs and Outputs initialization */
    gpio_pad_select_gpio(SLAVE_INPUT_0);
    gpio_set_direction(SLAVE_INPUT_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SLAVE_INPUT_0, GPIO_PULLUP_ONLY);
    gpio_pad_select_gpio(SLAVE_INPUT_1);
    gpio_set_direction(SLAVE_INPUT_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SLAVE_INPUT_1, GPIO_PULLUP_ONLY);
    gpio_pad_select_gpio(SLAVE_OUTPUT_2);
    gpio_set_direction(SLAVE_OUTPUT_2, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(SLAVE_OUTPUT_3);
    gpio_set_direction(SLAVE_OUTPUT_3, GPIO_MODE_OUTPUT);

    /* UART user interface task initialization */
    xTaskCreatePinnedToCore(&uart_interface_task,
    		"uart_interface",
			8192,
			NULL,
			6, // TODO Analyze the possible use of configMAX_PRIORITIES
			NULL,
			1);

    /* Slave task initialization */
	xTaskCreatePinnedToCore(&slave_task,
			"slave",
			8192,
			NULL,
			5,
			NULL,
			1);
}
