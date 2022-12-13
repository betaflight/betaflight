#pragma once

#include "pg/pg.h"
#include "common/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"   

#include "flight/imu.h"

#include "config/config.h"

//#include "flight/matrix.h"

/*卡尔曼滤波*/
typedef struct TWO_DIM_Matrix_s{
    float A11;
    float A12;
    float A21;
    float A22;
} TWO_DIM_Matrix_t;                      //定义2x2的矩阵
 
typedef struct Statue_s{
    float Altitude;                   //altitude
    float Alt_Velocity;                   //alt速度
}Statue_t;
 
typedef struct Statue_ERR_s{
    float W1;                        //状态高度误差
    float W2;                        //状态alt速度误差
}Statue_ERR_t;
 
typedef struct MEA_ERR_s{
    float V1;                        //测量高度误差
    float V2;                        //测量alt速度误差
}MEA_ERR_t;

typedef struct kalman_s
{
	TWO_DIM_Matrix_t Q; //控制误差 Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	TWO_DIM_Matrix_t R; //响应速度 R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
	TWO_DIM_Matrix_t A; //状态转移矩阵
	TWO_DIM_Matrix_t B; //输入矩阵
	TWO_DIM_Matrix_t H; //输出矩阵
	TWO_DIM_Matrix_t E; //单位阵
	TWO_DIM_Matrix_t P_last; //误差协方差上一时刻的值
	TWO_DIM_Matrix_t P_current; //误差协方差当前时刻值
	TWO_DIM_Matrix_t P_PRE; //误差协方差先验值
	TWO_DIM_Matrix_t Kp; //卡尔曼增益
	
	Statue_t X_last;  //上一时刻的状态值
	Statue_t X_current; //当前时刻的状态值
	Statue_t Z_last; //上一时刻测量输出值
	Statue_t Z_current; //当前时刻测量输出值
	Statue_t X_Hat_last; //上一时刻的最优值
	Statue_t X_Hat_current; //当前时刻最优值
	Statue_t X_Hat_PRE; //当前时刻的估计值
	Statue_ERR_t Process_Err_last; //上一时刻的过程误差
	Statue_ERR_t Process_Err_current; //当前时刻的过程误差
	MEA_ERR_t Measure_Err_last; //上一时刻的测量误差
	MEA_ERR_t Measure_Err_current; //当前时刻的测量误差

	float alt_throttle;

	/* data */
}kalman_t;

extern kalman_t kalman_alt;

void Status_Param_init(kalman_t *kalman);
void Mea_Param_init(kalman_t *kalman);
void TWO_DIM_Matrix_PARAM_init(kalman_t *kalman);

void Status_Param_estimation(kalman_t *kalman); // 状态估计
void Mea_Param_output(kalman_t *kalman); //输出值

void Kalman_init(void);

TWO_DIM_Matrix_t Matrix_Transpose(TWO_DIM_Matrix_t input_matrix);
TWO_DIM_Matrix_t Matrix_Congruent(TWO_DIM_Matrix_t input_matrix);
TWO_DIM_Matrix_t Matrix_Multi(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2);
TWO_DIM_Matrix_t Matrix_ADD(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2);
TWO_DIM_Matrix_t Matrix_Reduce(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2);
TWO_DIM_Matrix_t Inverse_Matrix(TWO_DIM_Matrix_t input_matrix);

void Kalman_Calc(kalman_t *kalman, float newMeasured, timeMs_t currentTimeMs);
void Update_Kalman(void);
float Get_alt_Kalman(void);
float Get_alt_Kalman_last(void);
float Get_finall_throttle(void);



/*定高控制过程*/
void updateAltHoldState(void); //先在接收部分判断接收的状态
void applyAltHold(void); //定高控制入口函数
void applyMultirotorAltHold(kalman_t *kalman); //利用控制器的结果进行油门的控制
bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude);
float calculateAltHoldThrottleAdjustment(kalman_t *kalman);//Alt and Velocity PID-Controler
void calculateEstimatedAltitude_kalman(timeUs_t currentTimeUs); //得到的卡尔曼滤波后的结果值


void kalmanFilter_init(kalman_t *kalman, float init_x, float init_p,float predict_q,float newMeasured_q);
void kalmanFilter_filter(kalman_t *kalman, float newMeasured);