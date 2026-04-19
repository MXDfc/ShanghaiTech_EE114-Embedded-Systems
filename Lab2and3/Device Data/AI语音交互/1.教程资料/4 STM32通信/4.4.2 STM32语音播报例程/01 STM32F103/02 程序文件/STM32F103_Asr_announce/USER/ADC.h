#ifndef _ADC_H_
#define _ADC_H_

#define ADC_BAT		13		//电池电压的AD检测通道
#define BUZZER 		PCout(13)
#define LED PBout(9)

void InitADC(void);
void InitLED(void);
void InitBuzzer(void);

void Buzzer(void);
void CheckBatteryVoltage(void);
u16 GetBatteryVoltage(void);
uint16 GetADCResult(BYTE ch);



#endif
