/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "socket.h"
#include "wizchip_conf.h"
#include "w5100s.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* CHIP SETTING */
#define _USE_W5100S_OPTIMIZE				1

#define W5100S_RESET_PIN		GPIO_PIN_8
#define W5100S_RESET_PORT		GPIOD
#define W5100S_CS_PIN			GPIO_PIN_7
#define W5100S_CS_PORT			GPIOD
#define W5100S_INT_PIN			GPIO_PIN_9
#define W5100S_INT_PORT			GPIOD

#define RESET_W5100S_GPIO_Port	GPIOD
#define RESET_W5100S_Pin		GPIO_PIN_8

/* SOCKET */
#define SOCKET_DHCP 0
#define SOCKET_LOOP 1

/* ETH */
#define ETH_MAX_BUF_SIZE		2048

/* DHCP */
enum
	{
	   DHCP_FAILED = 0,  ///< Processing Fail
	   DHCP_RUNNING,     ///< Processing DHCP protocol
	   DHCP_IP_ASSIGN,   ///< First Occupy IP from DHPC server      (if cbfunc == null, act as default default_ip_assign)
	   DHCP_IP_CHANGED,  ///< Change IP address by new ip from DHCP (if cbfunc == null, act as default default_ip_update)
	   DHCP_IP_LEASED,   ///< Stand by
	   DHCP_STOPPED      ///< Stop processing DHCP protocol
	};

#define DHCP_RETRY_COUNT 5

/* BUS */
#if 0
#define W5100S_BANK_ADDR                 ((uint32_t)0x60000000)//((uint32_t)0x64000000)
#define _W5100S_DATA(p)                  (*(volatile unsigned short*) (W5100S_BANK_ADDR + (p<<1)))
#endif


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* NET */
wiz_NetInfo gWIZNETINFO = {
		.mac = {0x00, 0x08, 0xdc, 0x6f, 0x00, 0x8a},
		.ip = {192, 168, 11, 101},
		.sn = {255, 255, 255, 0},
		.gw = {192, 168, 11, 1},
		.dns = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC
};

uint8_t ethBuf0[ETH_MAX_BUF_SIZE];

/* DHCP */
static uint8_t g_dhcp_get_ip_flag = 0;

/* UART */
uint8_t rxData[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void print_network_information(wiz_NetInfo* WIZNETINFO);


/* UART */
int _write(int fd, char *str, int len)
{
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    /*
        This will be called once data is received successfully,
        via interrupts.
    */

     /*
       loop back received data
     */
     HAL_UART_Receive_IT(&huart1, rxData, 1);
     HAL_UART_Transmit(&huart1, rxData, 1, 1000);
}

/* CHIP SETTING */
void wizchip_reset()
{
	HAL_GPIO_WritePin(RESET_W5100S_GPIO_Port, RESET_W5100S_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_W5100S_GPIO_Port, RESET_W5100S_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}
//for bus
#if 0
void W5100S_write(uint32_t addr, iodata_t wd)
{
	_W5100S_DATA(addr) = wd;
}
iodata_t W5100S_read(uint32_t addr)
{
	return _W5100S_DATA(addr);
}
#endif

void wizchip_check(void)
{
    /* Read version register */
    if (getVER() != 0x51) // W5100S
    {
        printf(" ACCESS ERR : VERSIONR != 0x51, read value = 0x%02x\n", getVER());
        while (1);
    }
}

void wizchip_initialize(void)
{
	uint8_t W5100S_AdrSet[2][4]= {{2,2,2,2},{2,2,2,2}};
    uint8_t tmp1, tmp2;
	intr_kind temp= IK_DEST_UNREACH;

	csEnable();
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*)W5100S_AdrSet) == -1)
	{
		printf(">>>>W5100s memory initialization failed\r\n");
	}
	if(ctlwizchip(CW_SET_INTRMASK,&temp) == -1)
	{
		printf("W5100S interrupt\r\n");
	}

	wizchip_check();
	while(1)
	{
		ctlwizchip(CW_GET_PHYLINK, &tmp1 );
		ctlwizchip(CW_GET_PHYLINK, &tmp2 );
		if(tmp1==PHY_LINK_ON && tmp2==PHY_LINK_ON) break;
	}
}

void print_network_information(wiz_NetInfo* WIZNETINFO)
{
    printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",WIZNETINFO->mac[0],WIZNETINFO->mac[1],WIZNETINFO->mac[2],WIZNETINFO->mac[3],WIZNETINFO->mac[4],WIZNETINFO->mac[5]);
    printf("IP address : %d.%d.%d.%d\n\r",WIZNETINFO->ip[0],WIZNETINFO->ip[1],WIZNETINFO->ip[2],WIZNETINFO->ip[3]);
    printf("SM Mask    : %d.%d.%d.%d\n\r",WIZNETINFO->sn[0],WIZNETINFO->sn[1],WIZNETINFO->sn[2],WIZNETINFO->sn[3]);
    printf("Gate way   : %d.%d.%d.%d\n\r",WIZNETINFO->gw[0],WIZNETINFO->gw[1],WIZNETINFO->gw[2],WIZNETINFO->gw[3]);
    printf("DNS Server : %d.%d.%d.%d\n\r",WIZNETINFO->dns[0],WIZNETINFO->dns[1],WIZNETINFO->dns[2],WIZNETINFO->dns[3]);
}

/* SPI */
uint8_t spiReadByte(void)
{
	uint8_t readByte=0;
	uint8_t writeByte=0xFF;

	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi2, &writeByte, &readByte, 1, 10);

	return readByte;
}

void spiWriteByte(uint8_t writeByte)
{
	uint8_t readByte=0;
	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi2, &writeByte, &readByte, 1, 10);
}
void spiReadBurst(uint8_t* pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi2, pBuf, len, 1000);
}

void spiWriteBurst(uint8_t* pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi2, pBuf, len, 1000);
}

void csEnable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_RESET);
}

void csDisable(void)
{
	HAL_GPIO_WritePin(W5100S_CS_PORT, W5100S_CS_PIN, GPIO_PIN_SET);
}

/* DHCP */
void wizchip_dhcp_assign(void)
{
    getIPfromDHCP(gWIZNETINFO.ip);
    getGWfromDHCP(gWIZNETINFO.gw);
    getSNfromDHCP(gWIZNETINFO.sn);
    getDNSfromDHCP(gWIZNETINFO.dns);

    ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
    printf("\r\n----------DHCP Net Information--------------\r\n");
    print_network_information(&gWIZNETINFO);
}

void wizchip_dhcp_update(void)
{
    ;
}

void wizchip_dhcp_conflict(void)
{
    ;
}

static void wizchip_dhcp_init(void)
{
    printf(" DHCP client running\n");

    DHCP_init(SOCKET_DHCP, ethBuf0);
    reg_dhcp_cbfunc(wizchip_dhcp_assign, wizchip_dhcp_assign, wizchip_dhcp_conflict);
}

void wizchip_dhcp_running(void)
{
    uint8_t retval = 0;
    uint8_t dhcp_retry = 0;

    retval = DHCP_run();
    if (retval == DHCP_IP_LEASED )
    {
    	if (g_dhcp_get_ip_flag == 0)
    	{
    		printf(" DHCP success\n");
    		g_dhcp_get_ip_flag = 1;
    	}
    }
    else if (retval == DHCP_FAILED)
    {
    	g_dhcp_get_ip_flag = 0;
    	dhcp_retry++;

    	if (dhcp_retry <= DHCP_RETRY_COUNT)
    	{
    		printf(" DHCP timeout occurred and retry %d\n", dhcp_retry);
    	}
    }

    if (dhcp_retry > DHCP_RETRY_COUNT)
    {
    	printf(" DHCP failed\n");
    	DHCP_stop();

    	while (1);
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  The call back function of ip assign.
 * @note
 * @param  None
 * @retval None
 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    wiz_NetInfo WIZNETINFO;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_RCC_GetSysClockFreq();
  HAL_UART_Receive_IT(&huart1, rxData, 1);

  wizchip_reset();
  csDisable();
#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_


  reg_wizchip_cs_cbfunc(csEnable,csDisable);// CS function register
  reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);// SPI method callback registration
  //reg_wizchip_spiburst_cbfunc(spiReadBurst, spiWriteBurst);// use DMA

#else
	// Indirect bus method callback registration
	//reg_wizchip_bus_cbfunc(busReadByte, busWriteByte);
#endif

  wizchip_initialize();

   if (gWIZNETINFO.dhcp == NETINFO_DHCP) // DHCP
   {
	   setSHAR(gWIZNETINFO.mac);
	   wizchip_dhcp_init();
   }
   else
   {
	   ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);

	   memset(&WIZNETINFO, 0x00, sizeof(wiz_NetInfo));
	   wizchip_getnetinfo(&WIZNETINFO);
	   print_network_information(&WIZNETINFO);
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Assigned IP through DHCP */
	  if (gWIZNETINFO.dhcp == NETINFO_DHCP)
	  {
		  wizchip_dhcp_running();
	  }

	  loopback_tcps(SOCKET_LOOP, ethBuf0, 3000);

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD8 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
