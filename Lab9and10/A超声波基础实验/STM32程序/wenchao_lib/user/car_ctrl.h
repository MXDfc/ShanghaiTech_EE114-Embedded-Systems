#ifndef _CAR_CTRL_H_
#define _CAR_CTRL_H_

#include "sysinc.h"

#define MOTOR_A_PWM PBout(7)
#define MOTOR_A_L PBout(8)
#define MOTOR_A_R PBout(9)

#define MOTOR_B_PWM PBout(6)
#define MOTOR_B_L PBout(5)
#define MOTOR_B_R PBout(4)

void car_init(void);
void car_forward(void);
void car_back(void);
void car_brake(void);

void car_turn_left_place(void);
void car_turn_right_place(void);
void car_turn_left(void);
void car_turn_right(void);

#endif

