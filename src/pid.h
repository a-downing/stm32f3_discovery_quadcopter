#ifndef PID_H
#define PID_H

#include <cmath>

struct PIDProperties
{
    float Kp;
    float Ki;
    float Kd;
    float i_limit;

    PIDProperties() = delete;
    PIDProperties(float Kp, float Ki, float Kd, float i_limit = 0.0f) : Kp(Kp), Ki(Ki), Kd(Kd), i_limit(i_limit) {}
};

class PIDController
{
    float Kp;
    float Ki;
    float Kd;
    float i_limit;
    float pv_last;
    float i_term;

public:
    PIDController() = delete;
    PIDController(float Kp, float Ki, float Kd, float i_limit = 0.0f) : Kp(Kp), Ki(Ki), Kd(Kd), i_limit(i_limit), pv_last(0.0f), i_term(0.0f) {}
    PIDController(const PIDProperties& prop) : Kp(prop.Kp), Ki(prop.Ki), Kd(prop.Kd), i_limit(prop.i_limit), pv_last(0.0f), i_term(0.0f) {}

    float control(float sp, float pv, float dt)
    {
        float error = sp - pv;

        float p_term = error * Kp;
        i_term += error * dt * Ki;
        float d_term = ((pv - pv_last) / dt) * Kd;

        if(i_limit > 0.0f)
        {
            if(fabsf(i_term) > i_limit)
            {
                i_term = (i_term > 0.0f) ? i_limit : -i_limit;
            }
        }

        pv_last = pv;

        return p_term + i_term + d_term;
    }
};

#endif