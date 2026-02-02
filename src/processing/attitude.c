#include "attitude.h"
#include <math.h>

void quaternion_init(quaternion_t *q)
{
    q->q0 = 1.0f;
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = 0.0f;
}

void quaternion_normalize(quaternion_t *q)
{
    float norm = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    if (norm > 0.0f) {
        q->q0 /= norm;
        q->q1 /= norm;
        q->q2 /= norm;
        q->q3 /= norm;
    }
}

void quaternion_update_rk2(quaternion_t *q, float omega_x, float omega_y, float omega_z, float dt)
{
    // 2nd-order Runge-Kutta integration
    // k1 = f(q, omega)
    float k1_q0 = 0.5f * (-q->q1*omega_x - q->q2*omega_y - q->q3*omega_z);
    float k1_q1 = 0.5f * ( q->q0*omega_x + q->q2*omega_z - q->q3*omega_y);
    float k1_q2 = 0.5f * ( q->q0*omega_y - q->q1*omega_z + q->q3*omega_x);
    float k1_q3 = 0.5f * ( q->q0*omega_z + q->q1*omega_y - q->q2*omega_x);

    // q_mid = q + k1 * dt/2
    quaternion_t q_mid;
    q_mid.q0 = q->q0 + k1_q0 * dt * 0.5f;
    q_mid.q1 = q->q1 + k1_q1 * dt * 0.5f;
    q_mid.q2 = q->q2 + k1_q2 * dt * 0.5f;
    q_mid.q3 = q->q3 + k1_q3 * dt * 0.5f;

    // k2 = f(q_mid, omega)
    float k2_q0 = 0.5f * (-q_mid.q1*omega_x - q_mid.q2*omega_y - q_mid.q3*omega_z);
    float k2_q1 = 0.5f * ( q_mid.q0*omega_x + q_mid.q2*omega_z - q_mid.q3*omega_y);
    float k2_q2 = 0.5f * ( q_mid.q0*omega_y - q_mid.q1*omega_z + q_mid.q3*omega_x);
    float k2_q3 = 0.5f * ( q_mid.q0*omega_z + q_mid.q1*omega_y - q_mid.q2*omega_x);

    // q = q + k2 * dt
    q->q0 += k2_q0 * dt;
    q->q1 += k2_q1 * dt;
    q->q2 += k2_q2 * dt;
    q->q3 += k2_q3 * dt;

    quaternion_normalize(q);
}

float quaternion_to_heading(const quaternion_t *q)
{
    // Extract yaw (heading) from quaternion
    // heading = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))
    return atan2f(2.0f * (q->q0*q->q3 + q->q1*q->q2),
                  1.0f - 2.0f * (q->q2*q->q2 + q->q3*q->q3));
}
