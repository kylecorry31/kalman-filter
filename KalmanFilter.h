#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
private:
    float e_est;
    float e_mea;
    float est;
    float q;
public:
    KalmanFilter(float e_mea, float e_est, float initial_est, float q);
    float filter(float measurement);
};

#endif
