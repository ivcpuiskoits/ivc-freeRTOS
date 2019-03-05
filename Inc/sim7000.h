#ifndef __sim7000_H
#define __sim7000_H

#ifdef __cplusplus
 extern "C" {
#endif

#define UART huart3
#include "main.h"

extern void Clr_Buff();
//extern int SIM7000_Init(void);
extern uint8_t SIM7000_Status(void);
extern int SIM7000_Init_GSM(void);
extern int SIM7000_SendData_PJU(float vrms, float irms, float power, float pf, float freq, float wh);
extern int SIM7000_SendData_tsk(unsigned char field, unsigned char data, unsigned short delay);
extern void SIM7000_ReadLastData_PJU(uint8_t *ststus, uint8_t *value);
extern float SIM7000_ReadLastData_tsk(unsigned int channel_id, unsigned char field);
extern uint64_t getIMI(void);
extern uint8_t getReply(const char *send, uint16_t timeout);
extern uint8_t getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude,
                              uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, float *sec);

#ifdef __cplusplus
}
#endif

#endif
