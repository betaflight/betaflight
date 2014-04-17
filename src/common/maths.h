
#pragma once

#ifndef sq
#define sq(x) ((x)*(x))
#endif

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f
#define RAD    (M_PI / 180.0f)

#define DEG2RAD(degrees) (degrees * RAD)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

typedef struct stdev_t
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

int constrain(int amt, int low, int high);

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float degreesToRadians(int16_t degrees);
