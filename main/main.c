#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lora.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

/* Private definitions and enumerations */
#define MASTERTX_SLAVERX_MSG_SIZE	2
#define NUM_OF_SLAVES				32
#define UART_RX_BUFFER_SIZE			256

typedef enum {
	TDMA_ST_MASTERTX_SLAVERX = 0,
	TDMA_ST_MASTERRX_SLAVETX,
	TDMA_ST_CHECK_NEXT_SLAVE,
	TDMA_ST_EXEC_USER_PROGRAM,
} tdma_state_t;

typedef struct {
	bool erase_address_cmd;
	bool available_slave;
	bool inputs[4];
	bool outputs[4];
} slave_ctrl_t;

/* Private variables */
uint8_t mastertx_msg[MASTERTX_SLAVERX_MSG_SIZE];
uint8_t masterrx_msg;

tdma_state_t tdma_state = TDMA_ST_MASTERTX_SLAVERX;
slave_ctrl_t slave_ctrl[NUM_OF_SLAVES] = {0};
bool initial_scan_done = false;
bool new_message_arrived = false;
bool set_new_slave_address_needed = false;
uint8_t new_slave_address = 0;

/* Private functions */
void lora_rx_done_callback(uint8_t* buffer_rx, int pac_size)
{
	memset(&masterrx_msg, 0, sizeof(masterrx_msg));
	memcpy(&masterrx_msg, buffer_rx, sizeof(masterrx_msg));
	new_message_arrived = true;
}

uint8_t get_next_available_slave (uint8_t current_slave_address)
{
	uint8_t next_available_slave_address = current_slave_address;

	/* Check which is the next available slave */
	do {
		next_available_slave_address++;
		if (next_available_slave_address >= NUM_OF_SLAVES)
			next_available_slave_address = 0;
		else if (next_available_slave_address == current_slave_address)
			return 0;
	} while( slave_ctrl[next_available_slave_address].available_slave == false );

	return next_available_slave_address;
}

bool is_erase_slave_address_needed (uint8_t slave_address)
{
	return slave_ctrl[slave_address].erase_address_cmd;
}

bool is_set_new_slave_address_needed ()
{
	return set_new_slave_address_needed;
}

void parse_io_cmd_message (uint8_t slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 0;
	uint16_t address_bits = ((uint16_t)slave_address) & 0x001F;
	uint16_t data_bits = 0;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[0])) & 0x0001) << 1;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[1])) & 0x0001) << 2;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[2])) & 0x0001) << 3;
	data_bits |= (((uint16_t)(slave_ctrl[slave_address].outputs[3])) & 0x0001) << 4;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_erase_address_cmd_message (uint8_t slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 1;
	uint16_t address_bits = ((uint16_t)slave_address) & 0x001F;
	uint16_t data_bits = 0;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_set_new_address_cmd_message (uint8_t new_slave_address, uint8_t* message)
{
	uint16_t raw_message = 0;

	uint16_t start_bit = 0;
	uint16_t command_bit = 0;
	uint16_t address_bits = 0;
	uint16_t data_bits = ((uint16_t)new_slave_address) & 0x001F;

	raw_message |= start_bit << 0;
	raw_message |= command_bit << 1;
	raw_message |= address_bits << 2;
	raw_message |= data_bits << 7;

	uint8_t number_of_ones = 0;
	for (int i = 0; i < 12; i++) {
		number_of_ones += (raw_message >> i) & 0x0001;
	}
	uint16_t parity_bit = number_of_ones % 2; // 0 if even, 1 if odd
	uint16_t end_bit = 1;
	raw_message |= parity_bit << 12;
	raw_message |= end_bit << 13;

	uint8_t lsb_message = (uint8_t)(raw_message & 0x00FF);
	uint8_t msb_message = (uint8_t)((raw_message >> 8) & 0x00FF);
	*message = lsb_message;
	message++;
	*message = msb_message;
}

void parse_received_message_from_slave (uint8_t slave_address, uint8_t message)
{
	uint8_t number_of_ones = 0;
	for (int i = 0; i < 5; i++) {
		number_of_ones += (message >> i) & 0x01;
	}
	uint8_t parity_bit_calculated = number_of_ones % 2;
	uint8_t parity_bit_from_msg = (message >> 5) & 0x01;
	if (parity_bit_from_msg == parity_bit_calculated) {
		slave_ctrl[slave_address].inputs[0] = (bool)((message >> 1) & 0x01);
		slave_ctrl[slave_address].inputs[1] = (bool)((message >> 2) & 0x01);
		slave_ctrl[slave_address].inputs[2] = (bool)((message >> 3) & 0x01);
		slave_ctrl[slave_address].inputs[3] = (bool)((message >> 4) & 0x01);
	}
}

void execute_user_program ()
{
	slave_ctrl[1].outputs[2] = slave_ctrl[2].inputs[0];
	slave_ctrl[1].outputs[3] = slave_ctrl[3].inputs[1];

	slave_ctrl[2].outputs[2] = slave_ctrl[3].inputs[0];
	slave_ctrl[2].outputs[3] = slave_ctrl[1].inputs[1];

	slave_ctrl[3].outputs[2] = slave_ctrl[1].inputs[0];
	slave_ctrl[3].outputs[3] = slave_ctrl[2].inputs[1];
}

void uart_interface_task(void *p)
{
	while (!initial_scan_done) {
		vTaskDelay(1);
	}

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
        uart_rxBytes = uart_read_bytes(UART_NUM_0, data, UART_RX_BUFFER_SIZE, 100 / portTICK_RATE_MS);
        if (uart_rxBytes > 0) {
            data[uart_rxBytes] = 0;

            if ( strncmp( "ERSADDR" , (char*)data , 7 ) == 0 ) {
            	int address_to_erase = atoi( (char*)(data + 7) );
            	/* Check if the address is valid (1 - 31) */
            	if ( (address_to_erase > 0) && (address_to_erase < NUM_OF_SLAVES) ) {
            		/* Check if the slave is available in the network */
            		if ( slave_ctrl[address_to_erase].available_slave ) {
            			/* Check if the slave 0 isn't available in the network */
            			if ( !(slave_ctrl[0].available_slave) ) {
            				slave_ctrl[address_to_erase].erase_address_cmd = true;
            				ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Address %i scheduled to be erased", address_to_erase);
            			}
            			else {
                            ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Address 00 is already available");
            			}
            		}
            		else {
                        ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Address not available");
            		}
            	}
            	else {
                    ESP_LOGI(uart_interface_task_tag, "Received Erase Address CMD: Address invalid");
            	}
            }
            else if ( strncmp( "SETADDR" , (char*)data , 7 ) == 0 ) {
            	int new_address_to_set = atoi( (char*)(data + 7) );
            	/* Check if the address is valid (1 - 31) */
            	if ( (new_address_to_set > 0) && (new_address_to_set < NUM_OF_SLAVES) ) {
            		/* Check if the slave isn't available in the network */
            		if ( !(slave_ctrl[new_address_to_set].available_slave) ) {
            			/* Check if the slave 0 is available in the network */
            			if (slave_ctrl[0].available_slave) {
            				new_slave_address = (uint8_t)new_address_to_set;
            				set_new_slave_address_needed = true;
            				ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address %i scheduled to be assigned", new_address_to_set);
            			}
            			else {
                            ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address 00 isn't available");
            			}
            		}
            		else {
                        ESP_LOGI(uart_interface_task_tag, "Received Set New Address CMD: Address already in use");
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
			if (slave_ctrl[0].available_slave) {
				printf("#SLV 00  # Use SETADDRXX to set new addr #\n");
				printf("##########################################\n");
			}
			for ( uint8_t i = 1 ; i < NUM_OF_SLAVES ; i++ ) {
				if (slave_ctrl[i].available_slave) {
					printf("#SLV %.2u  # %u # %u # %u # %u # %u # %u # %u # %u #\n",
							(unsigned int)i,
							(unsigned int)slave_ctrl[i].inputs[0],
							(unsigned int)slave_ctrl[i].inputs[1],
							(unsigned int)slave_ctrl[i].inputs[2],
							(unsigned int)slave_ctrl[i].inputs[3],
							(unsigned int)slave_ctrl[i].outputs[0],
							(unsigned int)slave_ctrl[i].outputs[1],
							(unsigned int)slave_ctrl[i].outputs[2],
							(unsigned int)slave_ctrl[i].outputs[3]);
					printf("##########################################\n");
				}
			}
			printf(LOG_RESET_COLOR);
		}

		vTaskDelay(1);
    }
}

void tdma_task(void *p)
{
	uint32_t tick_timeout = 0;
	uint8_t current_slave_adress = 0;

	/* Initial scan for detects available slaves */
	uint8_t slaves_founds = 0;
	printf(LOG_COLOR(LOG_COLOR_CYAN));
	printf("\nInitial scan for detecting available slaves\n");
	for ( uint8_t i = 0 ; i < NUM_OF_SLAVES ; i++ ) {
		new_message_arrived = false;
		memset(mastertx_msg, 0, sizeof(mastertx_msg));
		parse_io_cmd_message(i, mastertx_msg);
		lora_send_packet(mastertx_msg, sizeof(mastertx_msg));
		lora_receive();
		tick_timeout = xTaskGetTickCount();
		printf("Initial scan slave %u", (unsigned int)i);
		while (xTaskGetTickCount() - tick_timeout < pdMS_TO_TICKS(200)) {
			if (new_message_arrived) {
				slave_ctrl[i].available_slave = true;
				printf(" - available");
				slaves_founds++;
				break;
			}
			vTaskDelay(1);
		}
		printf("\n");
	}
	printf("Initial scan found %u slaves\n", (unsigned int)slaves_founds);
	printf(LOG_RESET_COLOR);
	initial_scan_done = true;

	for(;;) {
		switch (tdma_state) {
			case TDMA_ST_MASTERTX_SLAVERX:
				memset(mastertx_msg, 0, sizeof(mastertx_msg));
				new_message_arrived = false;

				/* Prepare the message to be transmitted to the slave N */
				if (current_slave_adress != 0) {
					/* Non-zero address slave */
					if (is_erase_slave_address_needed(current_slave_adress)) {
						parse_erase_address_cmd_message(current_slave_adress, mastertx_msg);
						slave_ctrl[0].available_slave = true;
					}
					else {
						parse_io_cmd_message(current_slave_adress, mastertx_msg);
					}
				}
				else {
					/* Zero address slave */
					if (is_set_new_slave_address_needed()) {
						parse_set_new_address_cmd_message(new_slave_address, mastertx_msg);
						slave_ctrl[0].available_slave = false;
						slave_ctrl[new_slave_address].available_slave = true;
					}
					else {
						parse_io_cmd_message(current_slave_adress, mastertx_msg);
					}
				}

				lora_send_packet(mastertx_msg, sizeof(mastertx_msg));

				lora_receive();
				tick_timeout = xTaskGetTickCount();
				tdma_state = TDMA_ST_MASTERRX_SLAVETX;
				break;

			case TDMA_ST_MASTERRX_SLAVETX:
				if (xTaskGetTickCount() - tick_timeout >= pdMS_TO_TICKS(100)) {
					/* Time to listen slave N transmission has expired
					 * Pass to the next slave available on the network */
					tdma_state = TDMA_ST_CHECK_NEXT_SLAVE;
				}
				else {
					/* Verify if the master has received a message
					 * If true, update the inputs table and then
					 * pass to the next slave available on the network */
					if (new_message_arrived) {
						if ( current_slave_adress != 0) {
							parse_received_message_from_slave(current_slave_adress, masterrx_msg);
						}
						else {
							slave_ctrl[0].available_slave = true;
						}
						tdma_state = TDMA_ST_CHECK_NEXT_SLAVE;
					}
				}
				break;

			case TDMA_ST_CHECK_NEXT_SLAVE:
				/* Verify if the scan is complete */
				if (get_next_available_slave(current_slave_adress) <= current_slave_adress) {
					tdma_state = TDMA_ST_EXEC_USER_PROGRAM;
				}
				else {
					tdma_state = TDMA_ST_MASTERTX_SLAVERX;
				}
				current_slave_adress = get_next_available_slave(current_slave_adress);
				break;

			case TDMA_ST_EXEC_USER_PROGRAM:
				execute_user_program();
				tdma_state = TDMA_ST_MASTERTX_SLAVERX;
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

    /* UART user interface task initialization */
    xTaskCreatePinnedToCore(&uart_interface_task,
    		"uart_interface",
			8192,
			NULL,
			6, // TODO Analyze the possible use of configMAX_PRIORITIES
			NULL,
			1);

    /* Time division multiple access task initialization */
	xTaskCreatePinnedToCore(&tdma_task,
			"tdma",
			8192,
			NULL,
			5,
			NULL,
			1);

//    EXAMPLE
//    slave_ctrl[0].available_slave = true;
//
//    slave_ctrl[5].available_slave = true;
//    slave_ctrl[5].inputs[1] = true;
//    slave_ctrl[5].outputs[2] = true;
//
//    slave_ctrl[27].available_slave = true;
//    slave_ctrl[27].inputs[2] = true;
//    slave_ctrl[27].outputs[3] = true;

}
