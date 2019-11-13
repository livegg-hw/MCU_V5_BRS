/*--------------------------------------------------------------------------------------------------------------*
 *  PROJECT    : LIVEGG                                                                                         *
 *  File name  : atwinc1500.c                                                                                   *
 *  Abstract   : ATWINC1500 handling driver.                                                                    *
 *  Written by : Ofer Freilich                                                                                  *
 *  Date       : JUNE 2018                                                                                      *
 *--------------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "../winc1500/driver/include/m2m_types.h"
#include "../winc1500/driver/include/m2m_wifi.h"
#include "../winc1500/socket/include/socket.h"
#include "client.h"
#include "eeprom_image.h"
#include "command.h"
#include "leds.h"
#include "battery.h"
#include "photo_diode.h"
#include "schedule.h"
#include "tray_id.h"

/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        				"TestLE" /* < Destination SSID  "TP-Link_5859" //  */
#define MAIN_WLAN_AUTH        				M2M_WIFI_SEC_WPA_PSK /* < Security manner */

#define MAIN_WLAN_PSK         				"kkfgkchmho" // "45154188"   //    /* < Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME  		"LIVEGG"
#define MAIN_WIFI_M2M_SERVER_IP      		0x0A2A0003 //0xFFFFFFFF /* 10.42.0.3 */
#define MAIN_WIFI_M2M_SERVER_PORT     		(10000)
//#define MAIN_WIFI_M2M_SERVER_IP      		0xc0a80164 //0xFFFFFFFF /* 255.255.255.255 */
//#define MAIN_WIFI_M2M_SERVER_PORT     	(6666)
#define MAIN_WIFI_M2M_REPORT_INTERVAL 		(1000)
#define CONNECTION_ATTEMPT_FAILED_TIMEOUT	115
#define MAIN_WIFI_M2M_BUFFER_SIZE   		1460

#define PERIPH_ENABLE						HAL_GPIO_WritePin(PERIPH_EN_GPIO_Port, PERIPH_EN_Pin, GPIO_PIN_SET  );
#define PERIPH_DISABLE						HAL_GPIO_WritePin(PERIPH_EN_GPIO_Port, PERIPH_EN_Pin, GPIO_PIN_RESET);
#define WIFI_ENABLE	    					HAL_GPIO_WritePin(GPIOE, WIFI_EN_Pin, GPIO_PIN_SET  );
#define WIFI_DISABLE    					HAL_GPIO_WritePin(GPIOE, WIFI_EN_Pin, GPIO_PIN_RESET);

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for client */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
volatile static uint8_t wifi_connected;
volatile uint8_t connection_attempt_failed_timer = 0;
struct sockaddr_in addr;

bool sending_ready = true;
uint16_t port_number = 9919; // 10000;
volatile bool wifi_sending_completed = false;
void isr(void);
extern TIM_HandleTypeDef htim17;

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) 
	{
    	case M2M_WIFI_RESP_CON_STATE_CHANGED:
    	{
    		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
    		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) 
    		{
    			LogPrintfWrapperC(FONT_GREEN, "wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
    			m2m_wifi_request_dhcp_client();
    		} 
    		else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) 
    		{
    			LogPrintfWrapperC(FONT_YELLOW,"wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
    			wifi_connected = 0;
    			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
    		}

    		break;
    	}

    	case M2M_WIFI_REQ_DHCP_CONF:
    	{
    		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
    		wifi_connected = 1;
    		LogPrintfWrapperC(FONT_YELLOW,"wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
    				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
    		break;
    	}

    	default:
    	{
    		break;
	    }
	}
}

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	wifi_sending_completed = true;
	
	switch (u8Msg) 
	{
    	/* Socket connected */
    	case SOCKET_MSG_CONNECT:
    	{
    		tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
    		if (pstrConnect && pstrConnect->s8Error >= 0) 
    		{
    			LogPrintfWrapperC(FONT_GREEN,"socket_cb: connect success!\r\n");
    			send(tcp_client_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0);
    			leds_illuminate_blinking( INDICATION_LED , GREEN , BLINKING_RATE_1_HZ , 50 );
    		} 
    		else 
    		{
    			LogPrintfWrapperC(FONT_YELLOW,"socket_cb: connect error!\r\n");
    			close(tcp_client_socket);
    			tcp_client_socket = -1;
    		}
    	}
    	break;

    	/* Message send */
    	case SOCKET_MSG_SEND:
    	{
    		sending_ready = true;
    		LogPrintfWrapperC(FONT_GREEN,"socket_cb: send success!\r\n");
    		recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
    		wifi_sending_completed = true;
    	}
    	break;

    	/* Message receive */
    	case SOCKET_MSG_RECV:
    	{
    		tstrSocketRecvMsg *pstrRecv __attribute__((aligned(8))) = (tstrSocketRecvMsg *)pvMsg;
    		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
    			client_receive_protocol( pstrRecv->pu8Buffer , pstrRecv->s16BufferSize );
    			LogPrintfWrapperC(FONT_GREEN,"socket_cb: recv success!\r\n");
    			LogPrintfWrapperC(FONT_GREEN,"TCP Client Test Complete!\r\n");
    		} else {
    			LogPrintfWrapperC(FONT_YELLOW,"socket_cb: recv error!\r\n");
    			close(tcp_client_socket);
    			tcp_client_socket = -1;
    		}
    	}

    	break;

    	default:
    		break;
	}
}

void atwinc1500_initialization()
{
	int8_t ret;
	static tstrWifiInitParam param;

	leds_illuminate( INDICATION_LED , ORANGE );
	leds_illuminate( CHARGE_LED     , battery_get_charging_led_color() );
	WIFI_ENABLE;
	#warning "Boris"
	port_number =  port_number_getting();
	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(port_number);
	addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) 
	{
		LogPrintfWrapperC(FONT_RED,"main: m2m_wifi_init call error!(%d)\r\n", ret);
	    schedule_goto_sleep();
//		while (1) {
//		}
	}

	__HAL_TIM_ENABLE(&htim17);
	__HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);

	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
}

void atwinc1500_connection_loop()
{
	static int8_t ret = -1;

	/* Handle pending events from network controller. */
	m2m_wifi_handle_events(NULL);

	if( connection_attempt_failed_timer >= CONNECTION_ATTEMPT_FAILED_TIMEOUT )
	{
		schedule_session_anomaly(WIFI_CONNECTION_FAIL);
	}
	if (wifi_connected == M2M_WIFI_CONNECTED)
	{
	    //send_message_to_pc( "wifi connected" , strlen("wifi connected") );
		/* Open client socket. */
		if (tcp_client_socket < 0)
		{
			if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
			{
				LogPrintfWrapperC(FONT_LIGHT_CYAN, "main: failed to create TCP client socket error!\r\n");
            }
			else
			{
				ret = 0;
			}
			
			if( ret >= 0 )
			{
				/* Connect server */
				 LogPrintfWrapperC(FONT_LIGHT_CYAN, "Connect to server\r\n");
				ret = connect(tcp_client_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));

				if (ret < 0)
				{

				    LogPrintfWrapperC(FONT_LIGHT_RED, "Connection fail\r\n");
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
		}
	}
}


void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
		isr();
	}
}

void set_wifi_port_number(int16_t new_port_number)
{
	port_number = new_port_number;
	////port_number_setting(new_port_number);
	//write_to_eeprom( EEPROM_PORT_NUMBER , port_number );
}

void get_wifi_port_number()
{
    LogPrintfWrapper("%d\n", port_number);
}

void atwinc1500_send_wifi_message( uint8_t *message , uint16_t message_length )
{
	uint16_t counter = 0;
	while( (sending_ready == false) && (counter++ < 100) );
	LogPrintfWrapperC(FONT_LIGHT_CYAN, "Send WiFi packet, len = %d\r\n", message_length);
	send(tcp_client_socket, message, message_length , 0 );
	sending_ready = false;
}

void atwinc1500_reset_connection_attempt_failed_timer()
{
	connection_attempt_failed_timer = 0;
}

void atwinc1500_stop_connection_attempt_failed_timer()
{
	__HAL_TIM_DISABLE(&htim17);
	__HAL_TIM_DISABLE_IT(&htim17 , TIM_IT_UPDATE);
}

uint32_t atwinc1500_get_ip_number()
{
	return MAIN_WIFI_M2M_SERVER_IP;
}
/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim17);
	//HAL_GPIO_TogglePin( GPIOC , GPIO_PIN_7 );
	//__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	connection_attempt_failed_timer++;
}
