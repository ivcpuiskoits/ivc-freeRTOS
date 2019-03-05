#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <stdbool.h>
#include "usart.h"
#include "sim7000.h"

uint8_t rx_data, rx_buffer[256], Transfer_cplt, datapos, dataqt;
uint32_t rx_index = 0;

uint8_t ok=0, ok1=0, try1=0, try2=0, var=0, ex=0;
//float lastdata;
char *pstr, *pch;
char keys[] = "1234567890";
char databuff[100], lastdatachar;
char databuff2[100];

char no[15];

char *tok;
char mystring [100];

//*** <<< move to main.h >>>    ***
//#define CHANNEL_ID	"481809"
//#define	USERAPIKEY	"5QXFUZQBEQLI73B3"
//#define WRITEAPIKEY	"0E7NG5ED1PMRU7X9"
//#define READAPIKEY	"Y00MR0DHWW562T3D"



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

//void sendUart(char uart[], char dat[], uint16_t cnt)
//{
//	uint16_t n;
//	char txBuff[512];
//	n=sprintf(txBuff,dat);
//	HAL_UART_Transmit(&UART, (uint8_t *)&txBuff, n, 2);
//}
void Clr_Buff()
{
	for(uint8_t i=0;i<=rx_index;i++)
			rx_buffer[i]=0;
	
	rx_index = 0;
	HAL_Delay(50);
}


uint8_t getReply(const char *send, uint16_t timeout)
{
	Clr_Buff();
	printf("%s\r",send);
	
	uint8_t l = *rx_buffer;
	return l;
}

/* Private function prototypes -----------------------------------------------*/
/*Inisialisasi SIM900A*/
//int SIM7000_Init()
//{
//	HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, 1);
//	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, 1);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, 1);
//	HAL_Delay(50);
//	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, 0);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, 1);
//	HAL_Delay(6000);
//	printf("AT\r");
//	HAL_Delay(130);
//	HAL_Delay(2000);
//	printf("AT\r");
//	HAL_Delay(2000);
//	ok = 1;
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
//}

uint8_t SIM7000_Status()
{
	printf("AT\r");
	HAL_Delay(130);
	do
	{
		Clr_Buff();
		printf("AT+CREG?\r");
		HAL_Delay(1000);
		ok++;
//		try1++;
//				pstr = strstr((char *)rx_buffer,"OK");
		return 0;
	}
	while(strstr((char *)rx_buffer,"1") == NULL);
	return 1;
}

int SIM7000_Init_GSM()
{
	try1=0;
	printf("AT\r");
	HAL_Delay(130);
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
			ok = 4;
			pstr = strstr((char *)rx_buffer,"OK");
			if(pstr != NULL)
			{ok = 5;}
		}
	}
}

int SIM7000_SendData_PJU(float vrms, float irms, float power, float pf, float freq, float wh)
{
	Clr_Buff();
	printf("AT+HTTPPARA=\"URL\",\"http://logging.abn.dx.am/inject_pju.php?vrms=%f&irms=%f&power=%f&pf=%f&freq=%f&wh=%f\"\r",vrms, irms, power, pf, freq, wh);
	HAL_Delay(1000);
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
		HAL_Delay(1000);
		ok=10;
	}
}

int SIM7000_SendData_tsk(uint8_t field, uint8_t data, uint16_t delay)
{
	Clr_Buff();
	printf("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/update?api_key=%s&field%d=%d\"\r",WRITEAPIKEY, field, data);
	HAL_Delay(1000);
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

void SIM7000_ReadLastData_PJU(uint8_t *ststus, uint8_t *value)
{
	Clr_Buff();
	printf("AT+HTTPPARA=\"URL\",\"http://logging.abn.dx.am/readpju.php\"\r");
	HAL_Delay(1000);
	ok=5;
	if(strstr((char *)rx_buffer,"OK") != NULL)
	{
		ok=6;
		do
		{
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
		Clr_Buff();
		printf("AT+HTTPREAD\r");
		HAL_Delay(500);
		
		// skip GPS run status
		char *tok = strtok(rx_buffer, ";");
		//if (! tok) return false;
		
		// grab ststus
    char *ststusp = strtok(NULL, ";");
    *ststus = atoi(ststusp);
		
		char *valuep = strtok(NULL, ";");
    *value = atoi(valuep);
	}
}

float SIM7000_ReadLastData_tsk(uint32_t channel_id, uint8_t field)
{
	float lastdata;
	Clr_Buff();
	HAL_Delay(1000);
	printf("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/channels/%d/fields/%d/last?api_key=%s\"\r",channel_id, field, WRITEAPIKEY);
	HAL_Delay(1000);
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
	return lastdata;
}

uint64_t getIMI()
{
	
	Clr_Buff();
	printf("AT+CIMI\r");
	HAL_Delay(1000);
	
	datapos = strcspn((char *)rx_buffer,keys);
	sprintf(no, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11], rx_buffer[datapos+12], rx_buffer[datapos+13], rx_buffer[datapos+14]);

//	uint64_t nom = atol(databuff);
	
}
uint64_t getIMI2(void)
{
	float lastdata;
	Clr_Buff();
	printf("AT+CIMI\r");
	HAL_Delay(1000);
	datapos = strcspn((char *)rx_buffer,keys);
	//sprintf(databuff, "%c", rx_buffer[datapos]);
	//dataqt = atoi(databuff);
	switch(datapos)
	{
		case 1: {sprintf(databuff, "%c", rx_buffer[datapos]);}
		break;
		case 2: {sprintf(databuff, "%c%c", rx_buffer[datapos], rx_buffer[datapos+1]);}
		break;
		case 3: {sprintf(databuff, "%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2]);}
		break;
		case 4: {sprintf(databuff, "%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3]);}
		break;
		case 5: {sprintf(databuff, "%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4]);}
		break;
		case 6: {sprintf(databuff, "%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5]);}
		break;
		case 7: {sprintf(databuff, "%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6]);}
		break;
		case 8: {sprintf(databuff, "%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7]);}
		break;
		case 9: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8]);}
		break;
		case 10: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9]);}
		break;
		case 11: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10]);}
		break;
		case 12: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11]);}
		break;
		case 13: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11], rx_buffer[datapos+12]);}
		break;
		case 14: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11], rx_buffer[datapos+12], rx_buffer[datapos+13]);}
		break;
		case 15: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11], rx_buffer[datapos+12], rx_buffer[datapos+13], rx_buffer[datapos+14]);}
		break;
		case 16: {sprintf(databuff, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", rx_buffer[datapos], rx_buffer[datapos+1], rx_buffer[datapos+2], rx_buffer[datapos+3], rx_buffer[datapos+4], rx_buffer[datapos+5], rx_buffer[datapos+6], rx_buffer[datapos+7], rx_buffer[datapos+8], rx_buffer[datapos+9], rx_buffer[datapos+10], rx_buffer[datapos+11], rx_buffer[datapos+12], rx_buffer[datapos+13], rx_buffer[datapos+14], rx_buffer[datapos+15]);}
		break;
	}
	lastdata = atoi(databuff);
}

uint8_t getGPS(float *lat, float *lon, float *speed_kph, 
							float *heading, float *altitude, uint16_t *year, 
							uint8_t *month, uint8_t *day, uint8_t *hour, 
							uint8_t *min, float *sec)
{
	Clr_Buff();
	printf("AT+CGNSPWR=1\r");
	HAL_Delay(1000);
	ok=1;
	if(strstr((char *)rx_buffer,"OK") != NULL)
	{
		ok=2;
		Clr_Buff();
		printf("AT+CGNSINF\r");
		ok=3;
		HAL_Delay(1000);

		// skip GPS run status
		char *tok = strtok(rx_buffer, ",");
		//if (! tok) return false;
		
		// skip fix status
    tok = strtok(NULL, ",");
		//if (! tok) return false;
		
      char *date = strtok(NULL, ",");
      //if (! date) return 0;
      
      // Seconds
      char *ptr = date + 12;
      *sec = atof(ptr);
      
      // Minutes
      ptr[0] = 0;
      ptr = date + 10;
      *min = atoi(ptr);

      // Hours
      ptr[0] = 0;
      ptr = date + 8;
      *hour = atoi(ptr);

      // Day
      ptr[0] = 0;
      ptr = date + 6;
      *day = atoi(ptr);

      // Month
      ptr[0] = 0;
      ptr = date + 4;
      *month = atoi(ptr);

      // Year
      ptr[0] = 0;
      ptr = date;
      *year = atoi(ptr);
		
		// grab the latitude
    char *latp = strtok(NULL, ",");
    //if (! latp) return false;

    // grab longitude
    char *longp = strtok(NULL, ",");
    //if (! longp) return false;

    *lat = atof(latp);
    *lon = atof(longp);

    // only grab altitude if needed
    if (altitude != NULL) {
      // grab altitude
      char *altp = strtok(NULL, ",");
      //if (! altp) return false;

      *altitude = atof(altp);
    }

    // only grab speed if needed
    if (speed_kph != NULL) {
      // grab the speed in km/h
      char *speedp = strtok(NULL, ",");
      //if (! speedp) return false;

      *speed_kph = atof(speedp);
    }

    // only grab heading if needed
    if (heading != NULL) {

      // grab the speed in knots
      char *coursep = strtok(NULL, ",");
      //if (! coursep) return false;

      *heading = atof(coursep);
    }

	}
}

