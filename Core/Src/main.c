/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/BSP/Camera/ov7725.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
	uint16_t r:5;
	uint16_t g:6;
	uint16_t b:5;
} RGB565Pix;

#define W 320
#define H 240
extern int DCMI_Line;
RGB565Pix framebuffer[H][W];
void Camera_XCLK_Stop(void);

void CameraTakePhoto(void)
{
	memset(framebuffer, 0, sizeof(framebuffer));
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uintptr_t)(void*)framebuffer, W*H/2);
	//while (hdcmi.State==HAL_DCMI_STATE_BUSY);
}

static const unsigned char base64_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void base64_encode(const unsigned char *src, char* dst, size_t len)
{
    unsigned char *pos = (unsigned char*)dst;
    const unsigned char *end, *in;

    size_t olen;

    olen = 4*((len + 2) / 3); /* 3-byte blocks to 4-byte */

    if (olen < len) {
    	*dst = 0;
        return; /* integer overflow */
    }
    end = src + len;
    in = src;
    while (end - in >= 3) {
        *pos++ = base64_table[in[0] >> 2];
        *pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
        *pos++ = base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
        *pos++ = base64_table[in[2] & 0x3f];
        in += 3;
    }

    if (end - in) {
        *pos++ = base64_table[in[0] >> 2];
        if (end - in == 1) {
            *pos++ = base64_table[(in[0] & 0x03) << 4];
            *pos++ = '=';
        }
        else {
            *pos++ = base64_table[((in[0] & 0x03) << 4) |
                (in[1] >> 4)];
            *pos++ = base64_table[(in[1] & 0x0f) << 2];
        }
        *pos++ = '=';
    }
}

void SIM800_TransmitPhoto(void)
{
	for (int i=0; i<H; i++)
	{
		uint8_t line[W];
		for (int j=0; j<W; j++)
		{
			uint8_t* pix = (uint8_t*)&framebuffer[i][j];
			uint8_t w = pix[0];
			pix[0] = pix[1];
			pix[1] = w;
			framebuffer[i][j] = *(RGB565Pix*)pix;
			line[j] = ((framebuffer[i][j].r<<3) + (framebuffer[i][j].g<<2) + (framebuffer[i][j].b<<3))/3;
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)line, sizeof(line), 10000);
	}
}

char reply[4096] = "\"10.10.10.10\"";
char sending[4096] = "\"10.10.10.10\"";
char cmd_buff[4096] = "\"10.10.10.10\"";
char ip[200] = "10.64.23.197";
int reply_code;

#define send_cmd(fmt, ...) (\
	sprintf(cmd_buff, fmt, ##__VA_ARGS__),\
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd_buff, strlen(cmd_buff), 1000),\
	reply_code=SIM800_GetReply(),\
	puts(reply),\
	reply_code\
)
#define SIM800_TCP_SendText(fmt, ...) (\
	sprintf(sending, fmt, ##__VA_ARGS__),\
	SIM800_TCP_SendBin(sending, strlen(sending)),\
	reply_code=SIM800_GetReply(),\
	puts(reply),\
	reply_code\
)

int SIM800_GetReply(void)
{
	memset(reply, 0, sizeof(reply));
	return SIM800_ReceiveUntilStr(&huart1, (uint8_t*)reply, sizeof(reply)-1, "OK\r\n", 2000)==HAL_TIMEOUT;
}

void SIM800_GPRS_Open(void)
{
	HAL_Delay(200);
	send_cmd("AT\r\n");
	send_cmd("ATE0\r\n");
	//send_cmd("ATE1\r\n");
	send_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
	send_cmd("AT+SAPBR=3,1,\"APN\",\"internet.mts.ru\"\r\n");
	send_cmd("AT+SAPBR=3,1,\"USER\",\"mts\"\r\n");
	send_cmd("AT+SAPBR=3,1,\"PWD\",\"mts\"\r\n");
	send_cmd("AT+SAPBR=1,1\r\n");
	send_cmd("AT+SAPBR=2,1\r\n");
	char* ip_start = strchr(reply, '"')+1;
	char* ip_end = strchr(ip_start, '"');
	memset(ip, 0, 17);
	uintptr_t diff = (uintptr_t)ip_end-(uintptr_t)ip_start;
	if (diff<18) {
		memcpy(ip, ip_start, diff);
	}
	printf("%s\r\n", ip);
}

void SIM800_GetDateTime(void)
{
	send_cmd("AT+CIPGSMLOC=2,1\r\n");
	char* date_begin = strchr(reply, ',')+1;
	int YYYY,MON,DD,HH,MIN,SS;
	printf("parsing: %s\n", reply);
	sscanf(date_begin, "%d/%d/%d,%d:%d:%d\r\n", &YYYY,&MON,&DD,&HH,&MIN,&SS);
	HH += 8;
	if (HH>=24) {
		HH = HH%24;
		DD++;
		if (DD>31) {
			if (MON>=12) {
				YYYY++;
				MON = 1;
			}
			DD = (DD%31)+1;
		}
		if (MON==4 || MON==6 || MON==9 || MON==11) {
			DD = (DD%30)+1;
		}
		if (MON==2) {
			if (YYYY%400==0 && YYYY%4==0 && YYYY%100!=0) {
				DD = (DD%29)+1;
			} else {
				DD = (DD%28)+1;
			}
		}
	}
	sprintf(reply, "%02d:%02d %02d.%02d.%04d", HH,MIN, DD,MON,YYYY);
}

void SIM800_GPRS_Close(void)
{
	send_cmd("AT+SAPBR=0,1\r\n");
}

void SIM800_TCP_Open(char* sip, short port)
{
	send_cmd("AT+SSLOPT=1,1\r\n");
	send_cmd("AT+CIPSSL=1\r\n");
	send_cmd("AT+CIPSTART=\"TCP\",\"%s\",\%d\r\n", sip, port);
	SIM800_ReceiveUntilStr(&huart1, (uint8_t*)reply, sizeof(reply)-1, "OK\r\n", 5000);
	puts(reply);
	send_cmd("AT+CIPSRIP=1\r\n");
}

int SIM800_TCP_SendBin(void* data, int length)
{
	for (int i=0; i<length; i+=1024)
	{
		int frame_size = 1024;
		if (i+1024>=length) {
			frame_size = length % 1024;
		}
		sprintf(cmd_buff, "AT+CIPSEND=%d\r\n", frame_size);
		HAL_UART_Transmit(&huart1, cmd_buff, strlen(cmd_buff), 10000);
		reply_code = SIM800_ReceiveUntilStr(&huart1, (uint8_t*)reply, sizeof(reply)-1, ">", 5000);
		printf(reply);
		if (reply_code) {
			return 1;
		}
		HAL_UART_Transmit(&huart1, data+i, frame_size, 10000);
		reply_code = SIM800_GetReply();
		printf(reply);
		if (reply_code) {
			return 1;
		}
	}
	return 0;
}

void SIM800_TCP_Close(void)
{
	puts("SIM800_TCP_Close");
	send_cmd("AT+CIPCLOSE\r\n");
	SIM800_ReceiveUntilStr(&huart1, (uint8_t*)reply, sizeof(reply)-1, "CLOSED\r\n", 5000);
	puts(reply);
}

_Bool check_invalid(void)
{
	if ((uintptr_t)strstr(reply, "Invalid") > (uintptr_t)reply)
	{
		return 1;
	}
	return 0;
}

void SIM800_SMTP_Send(char* from, char* to, char* msg)
{
	char plain_login[256] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
	SIM800_TCP_Open("smtp.mail.ru", 465);
	puts("EHLO");

	// Кидаем салют почтовому серверу
	SIM800_TCP_SendText("EHLO %d\r\n", ip);

	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Plain login");

	// Выполняем аутентификацию
	SIM800_TCP_SendText("AUTH PLAIN\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);

	// Здесь должен быть логин с паролем в base64
	SIM800_TCP_SendText("%s\r\n", plain_login);
	puts("Mail from");

	// Указываем отправителя
	SIM800_TCP_SendText("MAIL FROM: <%s>\r\n", from);
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Rcpt to");

	// Указываем получателя
	SIM800_TCP_SendText("RCPT TO: <%s>\r\n", to);
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Data");

	// Начинаем передавать данные
	SIM800_TCP_SendText("DATA\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("From");

	// Снова указываем отправителя и получателя
	SIM800_TCP_SendText("From: Mailbox <%s>\r\n", from);
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("To");
	SIM800_TCP_SendText("To: MailboxUser <%s>\r\n", to);
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Subject");

	// Тема письма
	SIM800_TCP_SendText("Subject: Physical mail\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Content-Type");

	// Тип контента HTML
	SIM800_TCP_SendText("Content-Type: text/html\r\n\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Message");

	// Отправка сообщения
	SIM800_TCP_SendText(msg);
	puts("End of message");

	// Завершение сообщения
	SIM800_TCP_SendText("\r\n.\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	puts("Close connection");

	// Закрываем соединение
	SIM800_TCP_SendText("QUIT\r\n");
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	SIM800_TCP_Close();
}

char pic_url[256] = "https://i.ibb.co/1qt8s7S/photo.png";
void SIM800_SendIMGBB(void)
{
	SIM800_GetDateTime();
	const char * head_fmt = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"image\"; filename=\"%s.bmp\"\r\nContent-Type: image/bmp\r\n\r\n";
	const char * tail = "\r\n--RandomNerdTutorials--\r\n"
			"Content-Disposition: form-data; name=\"album_id\"\r\n\r\n"
			"QbxmQR\r\n"
			"--RandomNerdTutorials--\r\n"
			"r\n";
	char head[strlen(head_fmt)+strlen(reply)];
	sprintf(head, head_fmt, reply);
	uint8_t line[W][3];
	int length = strlen(head) + strlen(tail) + 54 + sizeof(line)*H;

	// Создаём заголовок BMP
	uint8_t bmp_header[54];
	bmp_header[1] = 0x4D;
	bmp_header[0] = 0x42;
	*(uint32_t*)(bmp_header + 2) = 54 + W*H*3;
	*(uint16_t*)(bmp_header + 6) = 0;
	*(uint16_t*)(bmp_header + 8) = 0;
	*(uint32_t*)(bmp_header +10) = 54;
	*(uint32_t*)(bmp_header +14) = 40;
	*(uint32_t*)(bmp_header +18) = W;
	*(uint32_t*)(bmp_header +22) = H;
	*(uint16_t*)(bmp_header +26) = 1;
	*(uint16_t*)(bmp_header +28) = 24;
	*(uint32_t*)(bmp_header +30) = 0;
	*(uint32_t*)(bmp_header +34) = W*H*3;
	*(uint32_t*)(bmp_header +38) = 2000;
	*(uint32_t*)(bmp_header +42) = 2000;
	*(uint32_t*)(bmp_header +46) = 0;
	*(uint32_t*)(bmp_header +50) = 0;

	// Соединяемся с хостингом изображений
	SIM800_TCP_Open("api.imgbb.com", 443);
	SIM800_GetReply();
	// Отправляем POST запрос
	// Если не сработает, прописать expairation=600
	SIM800_TCP_SendText(
			"POST /1/upload?expiration=0&key=XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX HTTP/1.1\r\n"
			"Host: api.imgbb.com\r\nUser-Agent: curl/7.68.0\r\n"
			"Content-Type: multipart/form-data; boundary=RandomNerdTutorials\r\n"
			"Content-Length: %d\r\n\r\n", length);

	// Отправляем заголовок изображения в формате BMP
	SIM800_TCP_SendText(head);
	if (SIM800_TCP_SendBin(bmp_header, 54))
	{
		return;
	}
	HAL_Delay(1000);
	for (int i=0; i<H; i++)
	{
		for (int j=0; j<W; j++)
		{
			// Фотография идёт в формате RGB565, надо преобразовать в RGB888
			uint8_t* pix = (uint8_t*)&framebuffer[i][j];
			uint8_t w = pix[0];
			pix[0] = pix[1];
			pix[1] = w;
			framebuffer[i][j] = *(RGB565Pix*)pix;
			line[j][0] = (framebuffer[i][j].r<<3);
			line[j][1] = (framebuffer[i][j].g<<2);
			line[j][2] = (framebuffer[i][j].b<<3);
		}
		printf("img line %d\r\n", i);
		// Отправляем преобразованную строку изображения
		if (SIM800_TCP_SendBin((void*)line,sizeof(line)))
		{
			puts(reply);
			return;
		}
	}

	// Отправляем хвост
	if (SIM800_TCP_SendBin(tail, strlen(tail)))
	{
		puts(reply);
		return;
	}

	// Получаем JSON со ссылкой на загруженную фотографию
	SIM800_ReceiveUntilStr(&huart1, (uint8_t*)reply, sizeof(reply)-1, "}\r\n", 5000);
	puts(reply);

	// Ищём URL
	char* url_start = strstr(reply, "\"url\"");
	url_start = strchr(url_start, ':');
	url_start = strchr(url_start, '"')+1;
	char* url_end = strchr(url_start, '"');
	intptr_t url_length = (intptr_t)url_end - (intptr_t)url_start;
	printf("URL length is %d\r\n", (int)url_length);
	memset(pic_url, 0, sizeof(pic_url));
	int j=0;
	// Фильтруем URL от обратных слэшей
	for (int i=0; i<url_length; i++) {
		if (url_start[i]!='\\')
		{
			pic_url[j] = url_start[i];
			j++;
		}
	}

	// С UART творится что-то странное, в этом месте он перестаёт работать
	// нужно его перезапустить
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	printf("URL IS %s\r\n", pic_url);
	SIM800_TCP_Close();
	send_cmd("AT\r\n");
	send_cmd("ATE1\r\n");
}

// Направляем stdout через usb serial
int _write(int file, char *data, int len)
{
	CDC_Transmit_FS((uint8_t*)data, (uint16_t)len);
	// Слишком быстрая передача может привести к потере передаваемых данных
	// Поэтому делаем задержку
	HAL_Delay(1);
	return len;
}
int event = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  char html[1024];

  // Запускаем камеру
  Camera_XCLK_Set(XCLK_TIM);
  Camera_Init_Device(&hi2c1, FRAMESIZE_QVGA);
  // Отправка фотографии по USB
  /*while (1) {
	  while (1){
		  memset(html, 0, 2);
		  USBD_Interface_fops_FS.Receive((uint8_t*)html, (uint32_t*)&tick);
		  if (html[0]=='1') {
			  break;
		  }
	  }
	  uint16_t resol[3] = {W,H,3};
	  memset((void*)framebuffer, 0, sizeof(framebuffer));
	  CDC_Transmit_FS((uint8_t*)resol, 6);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_DCMI_Stop(&hdcmi);
	  CameraTakePhoto();

	  int g=1;
	  // Так как камера не может в 640x360 (самое ближайшее 640х480), а очень хочется, сначала отправляем нижнюю часть изображения
	  // При разрешении фотографии 640x480 и разрешении буфера в 640x360 происходит циклическое заполнение буфера
	  // и нижние 120 строк оказываются сверху.
	  for (int i=120; (i<=120)||g; i++)
	  {
		  uint8_t line[W][3];
		  for (int j=0; j<W; j++)
		  {
			uint8_t col[3];
			uint8_t* pix = (uint8_t*)&framebuffer[i][j];
			uint8_t w = pix[0];
			pix[0] = pix[1];
			pix[1] = w;
			framebuffer[i][j] = *(RGB565Pix*)pix;
			line[j][2] = (framebuffer[i][j].r<<3);
			line[j][1] = (framebuffer[i][j].g<<2);
			line[j][0] = (framebuffer[i][j].b<<3);
		  }
		  CDC_Transmit_FS((uint8_t*)line, sizeof(line));

		  HAL_Delay(2);
		  if (i==H-1)
		  {
			  i=0;
			  g=0;
		  }
	  }
  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Значение с датчика письма
	  int val = 0;
	  // Дельта значения
	  int delta = 0;
	  // Количество шагов дель
	  int steps = 50;
	  // Скользящее среднее дельты для шумоподавления
	  float smooth_delta = 0;
	  // Получаем значение с датчика
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  HAL_ADC_Stop(&hadc1);
	  // Цикл ожидания письма
	  while (1) {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1000);
		  HAL_ADC_Stop(&hadc1);
		  delta = HAL_ADC_GetValue(&hadc1) - val;

		  // вычисление скользящего среднего
		  smooth_delta = smooth_delta*0.9 + delta*0.1;
		  val = HAL_ADC_GetValue(&hadc1);
		  printf("ADC %d\n", (int)smooth_delta);
		  if (smooth_delta>30 && steps<=0) {
			  break;
		  }
		  steps--;
	  }
	  // Ждём, пока письмо упадёт
	  HAL_Delay(2000);

	  printf("Photographing\r\n");
	  // Включаем подсветку, всё-таки темно в ящике
	  HAL_GPIO_WritePin(CAM_LED_GPIO_Port, CAM_LED_Pin, 1);
	  // Ждём, пока камера подстроит экспозицию
	  HAL_Delay(100);
	  // Фотографируем
	  CameraTakePhoto();

	  // Выключаем подсветку - энергию надо экономить
	  HAL_GPIO_WritePin(CAM_LED_GPIO_Port, CAM_LED_Pin, 0);
	  printf("Starting GPRS\r\n");

	  // Выходим в интернет
	  SIM800_GPRS_Open();
	  printf("Sending photo to imgbb.com\r\n");

	  // Загружаем фотографию на imgbb.com
	  SIM800_SendIMGBB();

	  // Перезапускаем интернет, на всякий случай
	  SIM800_GPRS_Close();
	  SIM800_GPRS_Open();

	  // Формируем сообщение
	  sprintf(html, "<p>You've got a letter</p><img src=\"%s\" />\r\n", pic_url);
	  printf("Sending an e-mail\r\n");

	  // Отправляем сообщение
	  SIM800_SMTP_Send("отправитель@mail.ru", "получатель@gmail.com", html);

	  // Закрываем подключение GPRS
	  SIM800_GPRS_Close();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 150;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
