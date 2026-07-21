/*
 *******************************************************************************
 *                           Includes
 *******************************************************************************
 */
#include <math.h>

KALMAN Kalman_X;
KALMAN Kalman_E;


// 初始化卡尔曼滤波器
void init_kalman(KalmanState *state,float initial, KalmanConfig *config)
{
    state->x = initial;
    state->P = 1.0f; 
    config->Q = 0.10f;// 过程噪声协方阵（调整灵敏度）
	config->R = 0.10f; // 测量噪声协方阵（调整测量噪声）
}

// 预测步：根据时间间隔dt进行状态预测
void predict(KalmanState *state,KalmanConfig config,float dt)
{

    // 过程噪声协方阵：根据dt调整
    float Q = config.Q * dt;

	float temp_P=state->P + Q;
    // 更新协方阵
    state->P = temp_P;

}

// 更新步：当有测量值时更新状态
void update(KalmanState *state,KalmanConfig config,float measured)
{
    // 测量噪声协方阵
    float R = config.R;

    // 协方阵计算
    float S = state->P + R;

    // 卡尔曼增益
    float K = state->P / S;

    // 更新状态
    state->x = state->x + K* (measured - state->x);

    // 更新协方阵
    float temp_P=(1-K)*state->P;
	    // 更新协方阵
    state->P = temp_P;
}


void Kalman_Updata(KALMAN *Kalman,float val)
{
	if(Kalman->Klinit==0)
	{
		init_kalman(&Kalman->Klstate,val, &Kalman->Klconfig);
		Kalman->last_time = portGetTickCnt();
		Kalman->Klinit=1;
		return;
	}
	// 获取当前时间
	u32 current_time = portGetTickCnt();
	// 计算时间间隔dt（秒）
	float dt = (current_time - Kalman->last_time) / 1000.0f;
	// 更新上次时间
	Kalman->last_time = current_time;
	// 预测步
	predict(&Kalman->Klstate,Kalman->Klconfig,dt);
	update(&Kalman->Klstate,Kalman->Klconfig,val);
}
