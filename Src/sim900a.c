#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "sim900a.h"

uint8_t rx_data, rx_buffer[256], rx_index = 0, Transfer_cplt, datapos, dataqt;
uint8_t ok=0, ok1=0, try1=0, try2=0, var=0, ex=0;
char tx_buffer[512];
float lastdata;
char *pstr, *pch;
char keys[] = "1234567890";
char databuff[10], lastdatachar;



#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

	/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&UART, (uint8_t *)&ch, 1, 500);

    return ch;
}

void Clr_Buff()
{
	for(uint8_t i=0;i<=rx_index;i++)
			rx_buffer[i]=0;
	
	rx_index = 0;
	HAL_Delay(50);
}

/* Private function prototypes -----------------------------------------------*/
/*Inisialisasi SIM900A*/
int SIM900A_Init()
{
//	Clr_Buff();
	printf("AT\r");
	HAL_Delay(200);
//	pstr = strstr((char *)rx_buffer,"OK");
	if(strstr((char *)rx_buffer,"OK") != NULL)
	{
		ok = 1;
		Clr_Buff();
		printf("AT+SAPBR=3,1,\"APN\",\"internet\"\r");
		HAL_Delay(600);
//		pstr = strstr((char *)rx_buffer,"OK");
		if(strstr((char *)rx_buffer,"OK") != NULL)
		{
			ok = 2;
			do
			{
				Clr_Buff();
				printf("AT+SAPBR=1,1\r");
				HAL_Delay(1000);
				try1++;
//				pstr = strstr((char *)rx_buffer,"OK");
			}
			while(strstr((char *)rx_buffer,"OK") == NULL && try1 <= 3);
			ok = 3;
			Clr_Buff();
			printf("AT+HTTPINIT\r");
			HAL_Delay(200);
			Clr_Buff();
			printf("AT+HTTPPARA=\"CID\",1\r");
			HAL_Delay(200);
			pstr = strstr((char *)rx_buffer,"OK");
			if(pstr != NULL)
			{ok = 4;}
		}
	}
}

int SendData(uint8_t field, uint8_t data, uint16_t delay)
{
	Clr_Buff();
	printf("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/update?api_key=%s&field%d=%d\"\r",WRITEAPIKEY, field, data);
	HAL_Delay(200);
	ok=5;
	if(strstr((char *)rx_buffer,"OK") != NULL)
	{
		ok=6;
		do
		{
//			Clr_Buff();
//			printf("AT+SAPBR=1,1\r");
//			HAL_Delay(1000);

			Clr_Buff();
			printf("AT+HTTPACTION=0\r");
			ok=7;
			HAL_Delay(1000);
			
			uint16_t timeout = 1000;
			do
			{
				HAL_Delay(50);
				timeout--;
				ok=8;
			}
			while(strstr((char *)rx_buffer,"+HTTPACTION:") == NULL && timeout != 0);
			ok=9;
		}
		while(strstr((char *)rx_buffer,"200") == NULL);
		HAL_Delay(delay);
		ok=10;
	}
}

int ReadLastData(uint32_t channel_id, uint8_t field)
{
	Clr_Buff();
	printf("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/channels/%d/fields/%d/last?api_key=%s\"\r",channel_id, field, WRITEAPIKEY);
	HAL_Delay(5000);
	ok=5;
	if(strstr((char *)rx_buffer,"OK") != NULL)
	{
		ok=6;
		do
		{
//			Clr_Buff();
//			printf("AT+SAPBR=1,1\r");
//			HAL_Delay(1000);

			Clr_Buff();
			printf("AT+HTTPACTION=0\r");
			ok=7;
			HAL_Delay(1000);
			
			uint16_t timeout = 1000;
			do
			{
				HAL_Delay(50);
				timeout--;
				ok=8;
			}
			while(strstr((char *)rx_buffer,"+HTTPACTION:") == NULL && timeout != 0);
			ok=9;
		}
		while(strstr((char *)rx_buffer,"200") == NULL);
		HAL_Delay(100);
//		pch = strrchr((char *)rx_buffer,',');
//		posisidata = rx_buffer[pch-rx_buffer];
		Clr_Buff();
		printf("AT+HTTPREAD\r");
		HAL_Delay(500);
		datapos = strcspn((char *)rx_buffer,keys);
		sprintf(databuff, "%c", rx_buffer[datapos]);
		dataqt = atoi(databuff);
		switch(dataqt)
		{
			case 1: {sprintf(databuff, "%c", rx_buffer[datapos+3]);}
			break;
			case 2: {sprintf(databuff, "%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4]);}
			break;
			case 3: {sprintf(databuff, "%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5]);}
			break;
			case 4: {sprintf(databuff, "%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6]);}
			break;
			case 5: {sprintf(databuff, "%c%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7]);}
			break;
			case 6: {sprintf(databuff, "%c%c%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8]);}
			break;
			case 7: {sprintf(databuff, "%c%c%c%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9]);}
			break;
			case 8: {sprintf(databuff, "%c%c%c%c%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10]);}
			break;
			case 9: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c", rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11]);}
			break;
		}
		if(strstr(databuff, ".") != NULL)
		{lastdata = atof(databuff);}
		else
		{
			if(strspn(databuff, keys) != 0)
				lastdata = atoi(databuff);
		}
		ok=10;
	}
}

int ClearData()
{
	
}