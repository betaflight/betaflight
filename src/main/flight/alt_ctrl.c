#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

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
PID_TypeDef_t ALT_PID;
PID_TypeDef_t VEL_PID;

//static timeMs_t lastTimeMs = 0;
static timeMs_t currentTimeMs = 0;
// static timeUs_t currentTimeUs = 0;

float AltHold;  //设定高度
static float RcCommand_Throttle;

static float altHoldThrottleAdjustment = 0.0;        //定高油门调节量
static int16_t initialThrottleHold = 960;                  //定高时油门的初始值
static uint8_t velocityControl = 0;                  //是否利用速度环控制
static float setVelocity = 1.0;
static int32_t errorVelocityI = 0;
static float vel = 0.0f;

// static int16_t alt_pid[4] = {50, 0, 0, 0};
// static int16_t vel_pid[4] = {50, 0, 0, 0};


enum {
    DEBUG_ALTITUDE_ACC,
    DEBUG_ALTITUDE_VEL
};

void PID_ParamInit(PID_TypeDef_t *sPID, float LastError, float PrevError, float P, float I, float D, float Setpoint)
{
    sPID->LastError = LastError;
    sPID->PrevError = PrevError;
    sPID->P = P;
    sPID->I = I;
    sPID->D = D;
    sPID->SetPoint = Setpoint;
}

float SpdPIDCalc(PID_TypeDef_t *sPID, float NextPoint)
{
  float iError,iIncpid;
  iError = (float)sPID->SetPoint - NextPoint; //偏差

  /* 消除抖动误差 */
  if((iError<0.05f )&& (iError>-0.05f))
    iError = 0.0f;

  iIncpid=(sPID->P * iError)                //E[k]项
              -(sPID->I * sPID->LastError)     //E[k-1]项
              +(sPID->D * sPID->PrevError);  //E[k-2]项

  sPID->PrevError = sPID->LastError;                  //存储误差，用于下次计算
  sPID->LastError = iError;
  return(iIncpid);                                  //返回增量值

}


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


float calculateAltHoldThrottleAdjustment(kalman_t *kalman, PID_TypeDef_t *sPID) //Alt and Velocity PID-Controler
{
    float result = 0.0;
    float error = 0.0;
    float setVel = 0.0;

    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - kalman->X_Hat_current.Altitude, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((sPID->P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }

	// Velocity PID-Controller

    // P
    error = setVel - kalman->X_Hat_current.Alt_Velocity;
    result = constrain((sPID->P * error / 32), -300, +300);

    // I
    errorVelocityI += (sPID->I * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(sPID->D * (kalman->X_Hat_current.Altitude + kalman->X_Hat_last.Altitude) / 512, -150, 150);


    return result;
}

void calculateCurrentMeasure_kalman(kalman_t *kalman, timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);

    float accZ_tmp = 0;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
 //       const float dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

        // Integrator - velocity, cm/sec
        if (accSumCount) {
            accZ_tmp = (float)accSum[2] / accSumCount;
        }
        const float vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;
        vel = vel_acc;
    }
#endif

    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_ACC, accSum[2] / accSumCount);
    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_VEL, vel);


    kalman->Z_current.Alt_Velocity = accSum[2];
    kalman->Z_current.Altitude = rangefinderGetLatestAltitude();

    imuResetAccelerationSum();

 //   altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(&kalman_alt);
}
void calculateEstimatedAltitude_kalman(kalman_t *kalman, timeUs_t currentTimeUs) //获取卡尔曼滤波前的数据
{
    UNUSED(&kalman);
    UNUSED(currentTimeUs);
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

}

//获取高度后进行kalman滤波，必须是在新的数据到来以后才进行
void Kalman_Calc(kalman_t *kalman, timeMs_t currentTimeMs){

    UNUSED(currentTimeMs);
//	currentTimeMs = millis();

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

	kalman->P_last = kalman->P_current;
		
}

void Update_Kalman(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    calculateCurrentMeasure_kalman(&kalman_alt,currentTimeUs);
	Kalman_Calc(&kalman_alt, currentTimeMs);
}

float Get_Alt_Kalman(void)
{
	return kalman_alt.X_Hat_current.Altitude;
}

float Get_Vel_Kalman(void)
{
	return kalman_alt.X_Hat_current.Alt_Velocity;
}

float Get_Vel_measure(void)
{
    return kalman_alt.Z_current.Alt_Velocity;
}

float Get_alt_Kalman_last(void)
{
	return kalman_alt.X_Hat_last.Altitude;
}


float Get_finall_throttle(void)
{
	return kalman_alt.alt_throttle;
}
