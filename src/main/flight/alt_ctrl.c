#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "build/debug.h"

#include "flight/alt_ctrl.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

#include "common/time.h"

#include "config/config.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"


#define DEGREES_80_IN_DECIDEGREES 800
//状态位置过程误差
float E1[1] = {0.1};
//状态速度过程误差
float E2[1] = {0};
//测量位置过程误差
float E3[1] = {0.1};
//测量速度过程误差 
float E4[1] = {0};    

TWO_DIM_Matrix_t buffer;
kalman_t kalman_alt;

static timeMs_t lastTimeMs = 0;
static timeMs_t currentTimeMs;

float AltHold;  //设定高度
static float RcCommand_Throttle;

static float altHoldThrottleAdjustment = 0.0;        //定高油门调节量
static int16_t initialThrottleHold = 960;                  //定高时油门的初始值
static uint8_t velocityControl = 0;                  //是否利用速度环控制
static float setVelocity = 0.0;
static int32_t errorVelocityI = 0;

static int16_t alt_pid[4] = {50, 0, 0, 0};
static int16_t vel_pid[4] = {50, 0, 0, 0};

static float estimatedAltitude = 0.0;

void updateAltHoldState(void) //先在接收部分判断接收的状态  接收机以33HZ 高优先级接受模式的更改
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXRANGEFINDER)) {
        DISABLE_FLIGHT_MODE(RANGEFINDER_MODE);
        return;
    }
	else{
		ENABLE_FLIGHT_MODE(RANGEFINDER_MODE);
//		AltHold = estimatedAltitude;
        // altHoldThrottleAdjustment = 0;
		// initialThrottleHold = rcData[THROTTLE];
	}
}

void applyAltHold(void) //在taskMainPidLoop中进行定高实现，先把当前油门值记录下来，然后进行定高
{
    applyMultirotorAltHold(&kalman_alt);
}

void applyMultirotorAltHold(kalman_t *kalman) //利用控制器的结果进行油门的控制 
{
	RcCommand_Throttle = constrain(initialThrottleHold + altHoldThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;
	kalman->alt_throttle = constrainf(RcCommand_Throttle / 1000.0, 0.0f, 1.0f);
}

bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}


float calculateAltHoldThrottleAdjustment(kalman_t *kalman) //Alt and Velocity PID-Controler
{
    float result = 0.0;
    float error;
    float setVel;

    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - kalman->X_Hat_current.Altitude, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((alt_pid[0] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }

	// Velocity PID-Controller

    // P
    error = setVel - kalman->X_Hat_current.Altitude;
    result = constrain((vel_pid[0] * error / 32), -300, +300);

    // I
    errorVelocityI += (vel_pid[1] * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(vel_pid[2] * (kalman->X_Hat_current.Altitude + kalman->X_Hat_last.Altitude) / 512, -150, 150);


    return result;
}

void calculateEstimatedAltitude_kalman(timeUs_t currentTimeUs)  //得到的卡尔曼滤波后的结果值,自定义task（50Hz）进行最优高度的计算和油门值调节
{
    static timeUs_t previousTimeUs = 0;
    const uint32_t dTime = currentTimeUs - previousTimeUs;
	UNUSED(currentTimeUs);
	UNUSED(dTime);
	UNUSED(previousTimeUs);

#ifdef USE_ALT_HOLD
	estimatedAltitude = Get_alt_Kalman();
    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(&kalman_alt);
#endif
}

 
void Status_Param_init(kalman_t *kalman){  //状态以及过程误差初始化
	kalman->Process_Err_current.W1 = E1[0];
	kalman->Process_Err_current.W2 = E2[0];

	kalman->X_last.Altitude = 0.0;
	kalman->X_last.Alt_Velocity = 0.0;
}

void Status_Param_estimation(kalman_t *kalman) // 状态估计
{
	kalman->X_current.Altitude = kalman->A.A11*kalman->X_last.Altitude + kalman->A.A12*kalman->X_last.Alt_Velocity + kalman->Process_Err_last.W1;
	kalman->X_current.Alt_Velocity = kalman->A.A21*kalman->X_last.Altitude + kalman->A.A22*kalman->X_last.Alt_Velocity + kalman->Process_Err_last.W2;
}
 
void Mea_Param_init(kalman_t *kalman){  //测量误差和状态初始化
	kalman->Measure_Err_current.V1 = E3[0];
	kalman->Measure_Err_current.V2 = E4[0];	
	kalman->Z_last.Altitude = -1.0;
	kalman->Z_last.Alt_Velocity = 0.0;
}

void Mea_Param_output(kalman_t *kalman) //当前时刻的测量输出值
{
	kalman->Z_current.Altitude = rangefinderGetLatestAltitude();
	// kalman->Z_current.Alt_Velocity = ???;
	// kalman->Z_current.Altitude = kalman->H.A11*kalman->X_current.Altitude + kalman->H.A12*kalman->X_current.Alt_Velocity + kalman->Measure_Err_current.V1;
	// kalman->Z_current.Alt_Velocity = kalman->H.A21*kalman->X_current.Altitude + kalman->H.A22*kalman->X_current.Alt_Velocity + kalman->Measure_Err_current.V2;
}
 
void TWO_DIM_Matrix_PARAM_init(kalman_t *kalman){
 	//Q Matrix init
	kalman->Q.A11 = 0.1;	kalman->Q.A12 = 0.0;
	kalman->Q.A21 = 0.0;	kalman->Q.A22 = 0.1;	
   //R Matrix init
	kalman->R.A11 = 1.0;	kalman->R.A12 = 0.0;
	kalman->R.A21 = 0.0;	kalman->R.A22 = 1.0;
    //A Matrix init
	kalman->A.A11 = 1.0;	kalman->A.A12 = 0.0;
	kalman->A.A21 = 0.0;	kalman->A.A22 = 1.0;
	//B Matrix init
	kalman->B.A11 = 1.0;    kalman->B.A12 = 0.0;
	kalman->B.A21 = 0.0;	kalman->B.A22 = 1.0;
    //H Matrix init
	kalman->H.A11 = 1.0;	kalman->H.A12 = 0.0;
	kalman->H.A21 = 0.0;	kalman->H.A22 = 1.0;
    //E Matrix init
	kalman->E.A11 = 1.0;	kalman->E.A12 = 0.0;
	kalman->E.A21 = 0.0;	kalman->E.A22 = 1.0;	
  	//P Matrix init
	kalman->P_last.A11 = 1.0;	kalman->P_last.A12 = 0.0;
	kalman->P_last.A21 = 0.0;	kalman->P_last.A22 = 1.0;
}
//矩阵的转置运算
TWO_DIM_Matrix_t Matrix_Transpose(TWO_DIM_Matrix_t input_matrix){
    TWO_DIM_Matrix_t Result;
    Result.A11 = input_matrix.A11;
    Result.A12 = input_matrix.A21;
    Result.A21 = input_matrix.A12;
    Result.A22 = input_matrix.A22;
    return Result;
}
//矩阵的代数余子式
TWO_DIM_Matrix_t Matrix_Congruent(TWO_DIM_Matrix_t input_matrix){
    TWO_DIM_Matrix_t Result;
    Result.A11 = input_matrix.A22;
    Result.A12 = input_matrix.A12*(-1);
    Result.A21 = input_matrix.A21*(-1);
    Result.A22 = input_matrix.A11; 
    return Result;
}
//矩阵相乘
TWO_DIM_Matrix_t Matrix_Multi(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2){
    TWO_DIM_Matrix_t Result;
    Result.A11 = input_matrix1.A11*input_matrix2.A11 + input_matrix1.A12*input_matrix2.A21;
    Result.A12 = input_matrix1.A11*input_matrix2.A12 + input_matrix1.A12*input_matrix2.A22;
    Result.A21 = input_matrix1.A21*input_matrix2.A11 + input_matrix1.A22*input_matrix2.A21;
    Result.A22 = input_matrix1.A21*input_matrix2.A12 + input_matrix1.A22*input_matrix2.A22;
    return Result;
}
//矩阵相加
TWO_DIM_Matrix_t Matrix_ADD(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2){
    TWO_DIM_Matrix_t Result;
    Result.A11 = input_matrix1.A11 + input_matrix2.A11;
    Result.A12 = input_matrix1.A12 + input_matrix2.A12;
    Result.A21 = input_matrix1.A21 + input_matrix2.A21;
    Result.A22 = input_matrix1.A22 + input_matrix2.A22;
    return Result;
}
 
//矩阵相减
TWO_DIM_Matrix_t Matrix_Reduce(TWO_DIM_Matrix_t input_matrix1 , TWO_DIM_Matrix_t input_matrix2){
    TWO_DIM_Matrix_t Result;
    Result.A11 = input_matrix1.A11 - input_matrix2.A11;
    Result.A12 = input_matrix1.A12 - input_matrix2.A12;
    Result.A21 = input_matrix1.A21 - input_matrix2.A21;
    Result.A22 = input_matrix1.A22 - input_matrix2.A22;
    return Result;
}
//矩阵的逆矩阵
TWO_DIM_Matrix_t Inverse_Matrix(TWO_DIM_Matrix_t input_matrix){
    TWO_DIM_Matrix_t Result;
    float A;
    A = input_matrix.A11*input_matrix.A22 - input_matrix.A12*input_matrix.A21;
    A = 1.0/A;  
    Result = Matrix_Congruent(input_matrix);
    Result.A11 = A * Result.A11;
    Result.A12 = A * Result.A12;
    Result.A21 = A * Result.A21;
    Result.A22 = A * Result.A22; 
    return Result;
}

void Kalman_Calc_Init(kalman_t *kalman)
{
	kalman->X_Hat_last.Altitude = 0.0;
	kalman->X_Hat_last.Alt_Velocity = 0.0;
}

void Kalman_init(void)
{
	Status_Param_init(&kalman_alt);
	Mea_Param_init(&kalman_alt);
	TWO_DIM_Matrix_PARAM_init(&kalman_alt);
	Kalman_Calc_Init(&kalman_alt);

 //   kalmanFilter_init(&kalman_alt, -1, 0.1, 0.1, 0.5);
}

//获取高度后进行kalman滤波，必须是在新的数据到来以后才进行
void Kalman_Calc(kalman_t *kalman, float newMeasured, timeMs_t currentTimeMs){

    // UNUSED(currentTimeMs);
    UNUSED(newMeasured);
	kalman->Z_current.Altitude = rangefinderGetLatestAltitude();
	currentTimeMs = millis();
//    kalman->X_Hat_PRE.Alt_Velocity = 1;
	kalman->Z_current.Alt_Velocity = (kalman->Z_current.Altitude - kalman->Z_last.Altitude)*1000/(currentTimeMs - lastTimeMs);

	//预测过程  目前是没有输入的
	kalman->X_Hat_PRE.Altitude = kalman->A.A11*kalman->X_Hat_last.Altitude + kalman->A.A12*kalman->X_Hat_last.Alt_Velocity;
	kalman->X_Hat_PRE.Alt_Velocity = kalman->A.A21*kalman->X_Hat_last.Altitude + kalman->A.A22*kalman->X_Hat_last.Alt_Velocity;
	
	buffer = Matrix_Multi(kalman->A, kalman->P_last);
	buffer = Matrix_Multi(buffer, Matrix_Transpose(kalman->A));
	kalman->P_PRE = Matrix_ADD(buffer, kalman->Q);

	//更新过程
	buffer = Matrix_Multi(kalman->P_PRE, Matrix_Transpose(kalman->H));
	buffer = Inverse_Matrix(Matrix_ADD(Matrix_Multi(kalman->H, buffer), kalman->R));
	buffer = Matrix_Multi(Matrix_Transpose(kalman->H), buffer);
	kalman->Kp = Matrix_Multi(kalman->P_PRE, buffer);

	kalman->X_Hat_current.Altitude = kalman->X_Hat_PRE.Altitude + kalman->Kp.A11*(kalman->Z_current.Altitude - (kalman->H.A11*kalman->X_Hat_PRE.Altitude + kalman->H.A12*kalman->X_Hat_PRE.Alt_Velocity)) \
																+ kalman->Kp.A12*(kalman->Z_current.Alt_Velocity - (kalman->H.A21*kalman->X_Hat_PRE.Altitude + kalman->H.A22*kalman->X_Hat_PRE.Alt_Velocity));
	kalman->X_Hat_current.Alt_Velocity = kalman->X_Hat_PRE.Alt_Velocity + kalman->Kp.A21*(kalman->Z_current.Altitude - (kalman->H.A11*kalman->X_Hat_PRE.Altitude + kalman->H.A12*kalman->X_Hat_PRE.Alt_Velocity)) \
																		+ kalman->Kp.A22*(kalman->Z_current.Alt_Velocity - (kalman->H.A21*kalman->X_Hat_PRE.Altitude + kalman->H.A22*kalman->X_Hat_PRE.Alt_Velocity));

	kalman->P_current = Matrix_Multi(Matrix_Reduce(kalman->E, Matrix_Multi(kalman->Kp, kalman->H)), kalman->P_PRE);

	kalman->X_Hat_last.Altitude = kalman->X_Hat_current.Altitude;
	kalman->X_Hat_last.Alt_Velocity = kalman->X_Hat_current.Alt_Velocity;

	lastTimeMs = currentTimeMs; 

	kalman->P_last = kalman->P_current;
		
}

void Update_Kalman(void)
{
	Kalman_Calc(&kalman_alt, 0 ,currentTimeMs);
 //   kalmanFilter_filter(&kalman_alt, rangefinderGetLatestAltitude());
}

float Get_alt_Kalman(void)
{
	return kalman_alt.X_Hat_current.Altitude;
}


float Get_alt_Kalman_last(void)
{
	return kalman_alt.X_Hat_last.Altitude;
}

float Get_finall_throttle(void)
{
	return kalman_alt.alt_throttle;
}


void kalmanFilter_init(kalman_t *kalman, float init_x, float init_p,float predict_q,float newMeasured_q)
{
    kalman->X_Hat_last.Altitude = init_x;//待测量的初始值，如有中值一般设成中值
    kalman->P_last.A11 = init_p;//后验状态估计值误差协方差的初始值（不要为0问题不大）
    kalman->A.A11 = 1;
    kalman->H.A11 = 1;
    kalman->Q.A11 = predict_q;//预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
    kalman->R.A11 = newMeasured_q;//测量（观测）噪声方差R，可以通过实验手段获得
}

void kalmanFilter_filter(kalman_t *kalman, float newMeasured)
{
    /* Predict */
    kalman->X_Hat_PRE.Altitude = kalman->A.A11 * kalman->X_Hat_last.Altitude;//%x的先验估计由上一个时间点的后验估计值和输入信息给出
    kalman->P_PRE.A11 = kalman->A.A11 * kalman->A.A11 * kalman->P_last.A11 + kalman->Q.A11;  /*计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Correct */
    kalman->Kp.A11 = kalman->P_PRE.A11 * kalman->H.A11 / (kalman->P_PRE.A11 * kalman->H.A11 * kalman->H.A11 + kalman->R.A11);
    kalman->X_Hat_current.Altitude = kalman->X_Hat_PRE.Altitude + kalman->Kp.A11 * (newMeasured - kalman->H.A11 * kalman->X_Hat_PRE.Altitude);//利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
    kalman->P_current.A11 = (1 - kalman->Kp.A11 * kalman->H.A11) * kalman->P_PRE.A11;//%计算后验均方差

    kalman->X_Hat_last.Altitude = kalman->X_Hat_current.Altitude;
    kalman->P_last.A11 = kalman->P_current.A11;

 //   return kalman->X_current.Altitude;//得到现时刻的最优估计
}