/*
Copyright (C) 2017 Matt Harlum

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "esp_common.h"

// UART and GPIO drivers
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "i2c_bme280.h"
#include "../paho/include/MQTTESP8266.h"
#include "../paho/include/MQTTClient.h"
// For vTaskDelay
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// CONSTANTS 
#define AP_SSID "SSID"
#define AP_PASSWORD "PASSWORD"
#define MQTT_HOST "MQTT_HOST"
#define MQTT_PORT 1883
#define MQTT_USER "MQTT_USER"
#define MQTT_PASS "MQTT_PASS"
#define LED_GPIO 5
#define SDA_PIN 2
#define SCL_PIN 14
#define PUB_MSG_LEN 128
#define I2C_BUFFER_LEN 8


#define HDR_MARK 4400
#define HDR_SPACE 4300
#define BIT_MARK 543
#define ONE_SPACE 1623
#define ZERO_SPACE 472
#define RPT_MARK 440
#define RPT_SPACE 7048

const portTickType WAIT=5000/portTICK_RATE_MS;
const portTickType TASKDELAY=500/portTICK_RATE_MS;

xQueueHandle ir_queue;
xQueueHandle publish_queue;
xSemaphoreHandle wifi_alive;

s8 I2C_routine(void);
int8_t ICACHE_FLASH_ATTR i2c_writeByte(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length);
int8_t ICACHE_FLASH_ATTR i2c_readByte(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length);
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);

struct ir_packet 
{
	uint8_t size;
	uint8_t type;
	uint8_t data[10];
};

uint32 user_rf_cal_sector_set(void)
{
	flash_size_map size_map = system_get_flash_size_map();
	uint32 rf_cal_sec = 0;

	switch (size_map) {
		case FLASH_SIZE_4M_MAP_256_256:
			rf_cal_sec = 128 - 5;
			break;

		case FLASH_SIZE_8M_MAP_512_512:
			rf_cal_sec = 256 - 5;
			break;

		case FLASH_SIZE_16M_MAP_512_512:
		case FLASH_SIZE_16M_MAP_1024_1024:
			rf_cal_sec = 512 - 5;
			break;

		case FLASH_SIZE_32M_MAP_512_512:
		case FLASH_SIZE_32M_MAP_1024_1024:
			rf_cal_sec = 1024 - 5;
			break;

		default:
			rf_cal_sec = 0;
			break;
	}

	return rf_cal_sec;
}

// i2c
int8_t ICACHE_FLASH_ATTR i2c_writeByte(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length)
{
	i2c_start();
	if (!i2c_write(addr << 1)) {
		printf("i2c error!\r\n");
		i2c_stop();
		return 0;
	}
	i2c_write(reg);
	uint8_t loop;
	for (loop=0;loop < length; loop++)
	{
		if (!i2c_write(data[loop]))
		{
			printf("i2c error!\r\n");
			i2c_stop();
			return -1;
		}
	}
	i2c_stop();

	return 0;
}

int8_t ICACHE_FLASH_ATTR i2c_readByte(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length)
{
	i2c_start();
	if (!i2c_write(addr << 1)) {
		printf("i2c error!\r\n");
		i2c_stop();
		return -1;
	}
	i2c_write(reg);
	i2c_start();
	if (!i2c_write(((addr << 1) | 1))) {
		printf("i2c error!\r\n");
		i2c_stop();
		return -1;
	}
	uint8_t loop;
	for (loop=0;loop < length; loop++)
	{
		data[loop] = i2c_read();
		if (loop < (length-1))
		{
			i2c_set_ack(true);
		}
		else
		{
			i2c_set_ack(false);
		}
	}
	i2c_stop();

	return 0;
}


// Taks
void ICACHE_FLASH_ATTR get_temp(void * pvParameters)
{
	uint8_t tmp102_addr = 0x48;
	char msg[PUB_MSG_LEN];
	signed long int temp;
	unsigned long int press;
	unsigned long int hum;

	while (1)
	{
		uint8_t reg = 0;
		uint8_t data[2];
		i2c_readByte(tmp102_addr,reg,data,2);
		printf("HI: %02X LO: %02X\r\n", data[0], data[1]);
		int16_t temp = ((data[0] << 8) | (data[1]));
		temp >>= 4;
		if (temp & (1 << 11))
		{
			temp |= 0xF800;
		}
		temp *= 6.25;
		sprintf(msg, "weather,location=pcb temperature=%d", temp);
		xQueueSend(publish_queue, &msg, 0);
		BME280_readSensorData();
		temp = BME280_GetTemperature();
		press = BME280_GetPressure();
		hum = BME280_GetHumidity();
		snprintf(msg, PUB_MSG_LEN, "weather,location=loungeroom temperature=%d,pressure=%d,humidity=%d\n", temp, press, hum);
		printf("%s", msg);
		xQueueSend(publish_queue, &msg, 0);
		vTaskDelay(300000/portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

// IR Remote
void ICACHE_FLASH_ATTR mark(int time)
{
	int loop;
	for(loop=0;loop<(time/26);loop++)
	{
		// 38Khz 1/3 Duty cycle
		GPIO_OUTPUT_SET(LED_GPIO, false);
		os_delay_us(9);
		GPIO_OUTPUT_SET(LED_GPIO, true);
		os_delay_us(17);
	}
}

void ICACHE_FLASH_ATTR space(int time)
{
	os_delay_us( time );
}

void ICACHE_FLASH_ATTR send_ir(void * pvParameters)
{
	struct ir_packet *packet;
	while(1)
	{
			while(xQueueReceive(ir_queue, &packet, portMAX_DELAY))
			{
				printf("Sending codes...\n");
				mark(HDR_MARK);
				space(HDR_SPACE);
				uint8_t byte;
				int loop;
				for(loop=0;loop<2;loop++)
				{
					for(byte=0;byte<(packet->size);byte++)
					{
						printf("%02X", packet->data[byte]);
						uint8_t bit;
						for(bit=0;bit<8;bit++)
						{
							mark(BIT_MARK);
							if(0x80 & (packet->data[byte] << bit))
							{
									space(ONE_SPACE);
							}
							else
							{
									space(ZERO_SPACE);
							}
						}
					}
					printf("\n");
					mark(BIT_MARK);
					if(loop<1)
					{
						mark(RPT_MARK);
						space(RPT_SPACE);
					}
				}
			}
	}
	vTaskDelete(NULL);
}

LOCAL const char * ICACHE_FLASH_ATTR get_my_id(void)
{
	// Use MAC address for Station as unique ID
	static char my_id[13];
	static bool my_id_done = false;
	int8_t i;
	uint8_t x;
	if (my_id_done)
		return my_id;
	if (!wifi_get_macaddr(STATION_IF, my_id))
		return NULL;
	for (i = 5; i >= 0; --i)
	{
		x = my_id[i] & 0x0F;
		if (x > 9) x += 7;
		my_id[i * 2 + 1] = x + '0';
		x = my_id[i] >> 4;
		if (x > 9) x += 7;
		my_id[i * 2] = x + '0';
	}
	my_id[12] = '\0';
	my_id_done = true;
	return my_id;
}


// Callback when receiving subscribed message
LOCAL void ICACHE_FLASH_ATTR topic_received(MessageData* md)
{
	MQTTMessage* message = md->message;
	if(message->payloadlen <10)
	{
		struct ir_packet *packet;
		packet->size = message->payloadlen;
		memcpy(packet->data, message->payload, message->payloadlen);
		xQueueSend(ir_queue, (void *)&packet, portMAX_DELAY);
		printf("queued\r\n");
	}
}

LOCAL void ICACHE_FLASH_ATTR mqtt_task(void *pvParameters)
{
	int ret;
	struct Network network;
	MQTTClient client = DefaultClient;
	char mqtt_client_id[20];
	unsigned char mqtt_buf[100];
	unsigned char mqtt_readbuf[100];
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

	NewNetwork(&network);
	while (1)
	{
		xSemaphoreTake(wifi_alive, portMAX_DELAY);
		// Unique client ID
		strcpy(mqtt_client_id, "ESP-");
		strcat(mqtt_client_id, get_my_id());

		printf("(Re)connecting to MQTT server %s ... \n", MQTT_HOST);
		ret = ConnectNetwork(&network, MQTT_HOST, MQTT_PORT);
		if (!ret)
		{
			printf("ok.\r\n");
			NewMQTTClient(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);
			data.willFlag = 0;
			data.MQTTVersion = 3;
			data.clientID.cstring = mqtt_client_id;
			data.username.cstring = MQTT_USER;
			data.password.cstring = MQTT_PASS;
			data.keepAliveInterval = 10;
			data.cleansession = 0;
			printf("Send MQTT connect ...");
			ret = MQTTConnect(&client, &data);
			if (!ret)
			{
				printf("ok.\r\n");
				// Subscriptions
				MQTTSubscribe(&client, "/ir_remote", QOS1, topic_received);
				// Empty the publish queue
				xQueueReset(publish_queue);
				while (1)
				{
					// Publish all pending messages
					char msg[PUB_MSG_LEN];
					while (xQueueReceive(publish_queue, &msg, 0) == pdTRUE)
					{
						MQTTMessage message;
						message.payload = &msg;
						message.payloadlen = strlen(msg);
						message.dup = 0;
						message.qos = QOS0;
						message.retained = 1;
						ret = MQTTPublish(&client, "/influx_weather", &message);
						if (ret != SUCCESS)
							break;
					}
					// Receiving / Ping
					ret = MQTTYield(&client, 1000);
					if (ret == DISCONNECTED)
					{
						break;
					}
				}
				printf("Connection broken, request restart\r\n");
			}
			else
			{
				printf("failed.\r\n");
			}
			DisconnectNetwork(&network);
		}
		else
		{
			printf("failed.\r\n");
		}
	}
	printf("MQTT task ended\r\n");
	vTaskDelete(NULL);
}

LOCAL void ICACHE_FLASH_ATTR wifi_task(void *pvParameters)
{
	uint8_t status;

	if (wifi_get_opmode() != STATION_MODE)
	{
		wifi_set_opmode(STATION_MODE);
		vTaskDelay(1000 / portTICK_RATE_MS);
		system_restart();
	}

	while (1)
	{
		printf("WiFi: Connecting to WiFi\n");
		wifi_station_connect();
		struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
		sprintf(config->ssid, AP_SSID);
		sprintf(config->password, AP_PASSWORD);
		wifi_station_set_config(config);
		free(config);
		status = wifi_station_get_connect_status();
		int8_t retries = 30;
		while ((status != STATION_GOT_IP) && (retries > 0))
		{
			status = wifi_station_get_connect_status();
			if (status == STATION_WRONG_PASSWORD)
			{
				printf("WiFi: Wrong password\n");
				break;
			}
			else if (status == STATION_NO_AP_FOUND)
			{
				printf("WiFi: AP not found\n");
				break;
			}
			else if (status == STATION_CONNECT_FAIL)
			{
				printf("WiFi: Connection failed\n");
				break;
			}
			vTaskDelay(1000 / portTICK_RATE_MS);
			--retries;
		}
		if (status == STATION_GOT_IP)
		{
			printf("WiFi: Connected\n");
			vTaskDelay(5000 / portTICK_RATE_MS);
		}
		while ((status = wifi_station_get_connect_status()) == STATION_GOT_IP)
		{
			xSemaphoreGive(wifi_alive);
			// printf("WiFi: Alive\n");
			vTaskDelay(5000 / portTICK_RATE_MS);
		}
		printf("WiFi: Disconnected\n");
		wifi_station_disconnect();
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
	// Init UART0 and set baud rate to 115200 bps
	uart_init_new();
	printf("SDK version:%s\n", system_get_sdk_version());

	vSemaphoreCreateBinary(wifi_alive);
	xSemaphoreTake(wifi_alive,0);

	GPIO_AS_OUTPUT(LED_GPIO);
	GPIO_OUTPUT_SET(LED_GPIO, true);

	ir_queue = xQueueCreate(3, sizeof(struct ir_packet));
	publish_queue = xQueueCreate(3, PUB_MSG_LEN);
	i2c_init();
	BME280_Init(BME280_MODE_FORCED);
	// Create tasks
	xTaskCreate(send_ir, "send_ir", 256, NULL, 32, NULL);
	xTaskCreate(get_temp, "get_temp", 256, NULL, 3, NULL);
	xTaskCreate(wifi_task, "wifi", 256, NULL, 2, NULL);
	xTaskCreate(mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
}


