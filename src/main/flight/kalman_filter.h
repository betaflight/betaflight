#pragma once

#include "pg/pg.h"
#include "common/time.h"

#include "flight/Matrix.h"

/*卡尔曼滤波*/
typedef struct kalman_filter_s
{
	struct easyMatrix* A; //状态转移矩阵
	struct easyMatrix* B; //输入矩阵
	struct easyMatrix* H; //输出矩阵
	struct easyMatrix* E; //单位阵
	struct easyMatrix* Q; //控制误差 Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	struct easyMatrix* R; //响应速度 R:测量噪声，R增大，动态响应变慢，收敛稳定性变好

	struct easyMatrix* P_last; //误差协方差上一时刻的值
	struct easyMatrix* P_current; //误差协方差当前时刻值
	struct easyMatrix* P_PRE; //误差协方差先验值
	struct easyMatrix* Kp; //卡尔曼增益

	struct easyMatrix* Z_last; //上一时刻测量输出值
	struct easyMatrix* Z_current; //当前时刻测量输出值
	struct easyMatrix* X_Hat_last; //上一时刻的最优值
	struct easyMatrix* X_Hat_current; //当前时刻最优值
	struct easyMatrix* X_Hat_PRE; //当前时刻的估计值
	// struct easyMatrix* Process_Err_last; //上一时刻的过程误差
	// struct easyMatrix* Process_Err_current; //当前时刻的过程误差
	// struct easyMatrix* Measure_Err_last; //上一时刻的测量误差
	// struct easyMatrix* Measure_Err_current; //当前时刻的测量误差

	struct easyMatrix* U_last;
	struct easyMatrix* U_current;

	struct easyMatrix* buffer1;
	struct easyMatrix* buffer2;
	struct easyMatrix* buffer3;

	struct easyMatrix* buffer4;
	struct easyMatrix* buffer5;

	struct easyMatrix* buffer6;
	struct easyMatrix* buffer7;
	struct easyMatrix* buffer8;

	uint8_t alt_update;
	float trace;

	float dt;
	/* data */
}kalman_filter_t;

extern kalman_filter_t kalman_filter1;

void Create_Matrix(kalman_filter_t *kalman);
void Kalman_filter_init(void);
void calculate_Estimatedvel_acc(kalman_filter_t *kalman);

void Update_Kalman_filter(timeUs_t currentTimeUs);
void Kalman_Predicted(kalman_filter_t *kalman, timeUs_t currentTimeUs);
void Kalman_update(kalman_filter_t *kalman);
void Kalman_update_unalt(kalman_filter_t *kalman);

float Get_Alt_Kalman(void);
float Get_Alt_Pre_kalman(void);
float Get_Alt_Measure(void);
float Get_Vel_Kalman(void);
float Get_Trace_P_Current(void);
float Get_Acc_bias_kalman(void);
float Get_Trace_K_Current(void);