#ifndef __sim900a_H
#define __sim900a_H

#ifdef __cplusplus
 extern "C" {
#endif

extern int SIM900A_Init(void);
extern void Clr_Buff();
extern int SendData(unsigned char field, unsigned char data, unsigned short delay);
extern int ReadLastData(unsigned int channel_id, unsigned char field);
extern void gsmStatus(void);

#define UART huart3

#ifdef __cplusplus
}
#endif

#endif