#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "flight/kalman_filter.h"
#include "flight/Matrix.h"

#include "sensors/acceleration.h"


kalman_filter_t kalman_filter1;

float Zero[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float Zero31[] = {0, 0, 0};
float A1[] = {1.0, 0.002, 0, 0, 1.0, 1.0, 0, 0, 1.0}; // 1 0.002 0;0 1 1;0 0 1;
float B1[] = {0.0002, 0.002, 0};
float H1[] = {1.0, 0, 0, 0, 0, 0, 0, 0, 0}; // 1 0 0;0 0 0;0 0 0; 
float Q1[] = {0.1, 0, 0, 0, 1, 0, 0, 0, 1};
float R1[] = {0.003, 0, 0, 0, 1, 0, 0, 0, 1};
float P_last1[] = {10.0, 0, 0, 0, 10.0, 0, 0, 0, 10.0};
float P_current1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float P_PRE1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float X_last1[] = {0, 0, 10};
float E1[] = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
float Kp1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float U1[] = {0};


void Create_Matrix(kalman_filter_t *kalman)
{
    kalman->A = Creat_Matrix(3,3,A1);
    kalman->B = Creat_Matrix(3,1,B1);
    kalman->H = Creat_Matrix(3,3,H1);
//    kalman->H = Creat_Matrix(1,3,H1);
    kalman->E = Creat_Matrix(3,3,E1);
    kalman->Q = Creat_Matrix(3,3,Q1);
    kalman->R = Creat_Matrix(3,3,R1);
//    kalman->R = Creat_Matrix(1,1,R1);

    kalman->P_last = Creat_Matrix(3,3,P_last1);
    kalman->P_PRE  = Creat_Matrix(3,3,P_PRE1);
    kalman->P_current = Creat_Matrix(3,3,P_current1);

    kalman->Kp = Creat_Matrix(3,3,Kp1);

    kalman->X_Hat_last = Creat_Matrix(3,1,X_last1);
    kalman->X_Hat_PRE  = Creat_Matrix(3,1,Zero31);
    kalman->X_Hat_current = Creat_Matrix(3,1,Zero31);

    kalman->Z_last = Creat_Matrix(3,1,Zero31);
    kalman->Z_current = Creat_Matrix(3,1,Zero31);

    kalman->U_last = Creat_Matrix(1,1,U1);
    kalman->U_current = Creat_Matrix(1,1,U1);

    kalman->buffer1 = Creat_Matrix(3,3,Zero);
    kalman->buffer2 = Creat_Matrix(3,3,Zero);
    kalman->buffer3 = Creat_Matrix(3,3,Zero);

    kalman->buffer4 = Creat_Matrix(3,1,Zero31);
    kalman->buffer5 = Creat_Matrix(3,1,Zero31);

    kalman->buffer6 = Creat_Matrix(1,3,Zero31);
    kalman->buffer7 = Creat_Matrix(1,1,U1);
    kalman->buffer8 = Creat_Matrix(1,1,U1);

}

uint8_t scale;

void calculate_Estimatedvel_acc(kalman_filter_t *kalman)
{
    float acc_altitude;
    float x;
    float y;
    float z;

    x = (acc.accADC[X]/ scale) * 0.001953125f;
    y = (acc.accADC[Y]/ scale) * 0.001953125f;
    z = (acc.accADC[Z]/ scale) * 0.001953125f;

    acc_altitude = sqrtf(sq(x) + sq(y) + sq(z));
    kalman->U_current->element[0] = (acc_altitude - 1.0f)*9.80665f;

}

void Kalman_filter_init(void)
{
	Create_Matrix(&kalman_filter1);
    if (acc.dev.acc_1G > 512 * 4) {
        scale = 8;
    } else if (acc.dev.acc_1G > 512 * 2) {
        scale = 4;
    } else if (acc.dev.acc_1G >= 512) {
        scale = 2;
    } else {
        scale = 1;
    }

}

void Kalman_Predicted(kalman_filter_t *kalman, timeUs_t currentTimeUs)
{
    //预测过程
    static timeUs_t lastTimeUs = 0;
    const float dTime = (currentTimeUs - lastTimeUs)*1e-6f;

    kalman->dt = dTime;
    kalman->A->element[1] = dTime;
    kalman->A->element[2] = 0.5*dTime*dTime;
    kalman->A->element[5] = dTime;
    kalman->B->element[0] = 0.5*dTime*dTime;
    kalman->B->element[1] = dTime;
    kalman->B->element[2] = 0;

    multiMatrix(kalman->A,kalman->X_Hat_last,kalman->buffer4);
    multiMatrix(kalman->B,kalman->U_last,kalman->buffer5);
    addMatrix(kalman->buffer4,kalman->buffer5,kalman->X_Hat_PRE);

    multiMatrix(kalman->A,kalman->P_last,kalman->buffer1);
    transMatrix(kalman->A,kalman->buffer2);
    multiMatrix(kalman->buffer1,kalman->buffer2,kalman->buffer3);
    addMatrix(kalman->buffer3,kalman->Q,kalman->P_PRE);

    lastTimeUs = currentTimeUs;

}

void Kalman_update(kalman_filter_t *kalman)
{
    //更新过程
    multiMatrix(kalman->H,kalman->P_PRE,kalman->buffer1);
    transMatrix(kalman->H,kalman->buffer2);
    multiMatrix(kalman->buffer1,kalman->buffer2,kalman->buffer3);
    addMatrix(kalman->buffer3,kalman->R,kalman->buffer1);
    invMatrix(kalman->buffer1,kalman->buffer3);
    multiMatrix(kalman->buffer2,kalman->buffer3,kalman->buffer1);
    multiMatrix(kalman->P_PRE,kalman->buffer1,kalman->Kp);
    
    multiMatrix(kalman->H,kalman->X_Hat_PRE,kalman->buffer4);
    subMatrix(kalman->Z_current,kalman->buffer4,kalman->buffer5);
    multiMatrix(kalman->Kp,kalman->buffer5,kalman->buffer4);
    addMatrix(kalman->X_Hat_PRE,kalman->buffer4,kalman->X_Hat_current);

    multiMatrix(kalman->Kp,kalman->H,kalman->buffer1);
    subMatrix(kalman->E,kalman->buffer1,kalman->buffer2);
    multiMatrix(kalman->buffer2,kalman->P_PRE,kalman->P_current);

}

// void Kalman_Predicted(kalman_filter_t *kalman, timeUs_t currentTimeUs)
// {
//     //预测过程
//     static timeUs_t lastTimeUs = 0;
//     const float dTime = (currentTimeUs - lastTimeUs)*1e-6f;

//     kalman->dt = dTime;
//     kalman->A->element[1] = dTime;
//     kalman->A->element[2] = 0.5*dTime*dTime;
//     kalman->A->element[5] = dTime;
//     kalman->B->element[0] = 0.5*dTime*dTime;
//     kalman->B->element[1] = dTime;
//     kalman->B->element[2] = 0;

//     multiMatrix(kalman->A,kalman->X_Hat_last,kalman->buffer4);
//     multiMatrix(kalman->B,kalman->U_last,kalman->buffer5);
//     addMatrix(kalman->buffer4,kalman->buffer5,kalman->X_Hat_PRE);

//     multiMatrix(kalman->A,kalman->P_last,kalman->buffer1);
//     transMatrix(kalman->A,kalman->buffer2);
//     multiMatrix(kalman->buffer1,kalman->buffer2,kalman->buffer3);
//     addMatrix(kalman->buffer3,kalman->Q,kalman->P_PRE);

//     lastTimeUs = currentTimeUs;

// }

// void Kalman_update(kalman_filter_t *kalman)
// {
//     //更新过程
//     multiMatrix(kalman->H,kalman->P_PRE,kalman->buffer6);
//     transMatrix(kalman->H,kalman->buffer4);
//     multiMatrix(kalman->buffer6,kalman->buffer4,kalman->buffer7);
//     addMatrix(kalman->buffer7,kalman->R,kalman->buffer8);
//     invMatrix(kalman->buffer8,kalman->buffer7);
//     multiMatrix(kalman->buffer4,kalman->buffer7,kalman->buffer5);
//     multiMatrix(kalman->P_PRE,kalman->buffer5,kalman->Kp);
    
//     multiMatrix(kalman->H,kalman->X_Hat_PRE,kalman->buffer7);
//     //subMatrix(kalman->Z_current,kalman->buffer4,kalman->buffer5);
//     kalman->buffer8->element[0] = kalman->Z_current->element[0] - kalman->buffer7->element[0];
//     multiMatrix(kalman->Kp,kalman->buffer8,kalman->buffer4);
//     addMatrix(kalman->X_Hat_PRE,kalman->buffer4,kalman->X_Hat_current);

//     multiMatrix(kalman->Kp,kalman->H,kalman->buffer1);
//     subMatrix(kalman->E,kalman->buffer1,kalman->buffer2);
//     multiMatrix(kalman->buffer2,kalman->P_PRE,kalman->P_current);

// }



void Kalman_update_unalt(kalman_filter_t *kalman)
{
    copyMatrix(kalman->X_Hat_PRE,kalman->X_Hat_current);
    copyMatrix(kalman->P_PRE,kalman->P_current);
}

void Update_Kalman_filter(timeUs_t currentTimeUs)
{
    Kalman_Predicted(&kalman_filter1, currentTimeUs);
    
    if(kalman_filter1.alt_update == 1){
 //       kalman_filter1.Z_current->element[2] = 0.2;
        Kalman_update(&kalman_filter1);
        kalman_filter1.alt_update = 0;
    }
    else{
        Kalman_update_unalt(&kalman_filter1);
    }

    calculate_Estimatedvel_acc(&kalman_filter1);
    // kalman_filter1.trace = TraceMatrix(kalman_filter1.Kp);
    //kalman_filter1.trace = kalman_filter1.Kp->element[0] + kalman_filter1.Kp->element[1] + kalman_filter1.Kp->element[2];
    kalman_filter1.trace = TraceMatrix(kalman_filter1.P_current);
    copyMatrix(kalman_filter1.X_Hat_current,kalman_filter1.X_Hat_last);
    copyMatrix(kalman_filter1.P_current,kalman_filter1.P_last);
    copyMatrix(kalman_filter1.U_current,kalman_filter1.U_last);

    // kalman_filter1.X_Hat_last = kalman_filter1.X_Hat_current;
    // kalman_filter1.P_last = kalman_filter1.P_current;
    
}


float Get_Alt_Kalman(void)
{
	return kalman_filter1.X_Hat_current->element[0];
}

float Get_Alt_Pre_kalman(void)
{
    return kalman_filter1.X_Hat_PRE->element[0];
}

float Get_Alt_Measure(void)
{
    return kalman_filter1.Z_current->element[0];
}

float Get_Vel_Kalman(void)
{
	return kalman_filter1.X_Hat_current->element[1];
}

float Get_Acc_bias_kalman(void)
{
    return kalman_filter1.X_Hat_current->element[2];
}

float Get_Trace_P_Current(void)
{
    return kalman_filter1.trace;
}

float Get_Trace_K_Current(void)
{
    return kalman_filter1.trace;
}