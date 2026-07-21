/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef __KALMAN_H__
#define __KALMAN_H__

#include  "main.h"

// 状态向量：[distance, velocity]
typedef struct {
    float x;
    float P; // 状态协方阵
} KalmanState;

// 卡尔曼滤波参数
typedef struct {
    float Q;         // 过程噪声协方阵（标量，简化）
    float R;         // 测量噪声协方阵（标量，简化）
} KalmanConfig;

typedef struct
{
	KalmanState Klstate;
	KalmanConfig Klconfig;
	u32        last_time;
	u8         Klinit;
}KALMAN;

extern KALMAN Kalman_X;
extern KALMAN Kalman_E;

void init_kalman(KalmanState *state,float initial, KalmanConfig *config);
void predict(KalmanState *state,KalmanConfig config,float dt);
void update(KalmanState *state,KalmanConfig config,float measured);
void Kalman_Updata(KALMAN *Kalman,float val);
#endif




