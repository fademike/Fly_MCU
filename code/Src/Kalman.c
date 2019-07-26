/*
 * Kalman.c
 *
 *  Created on: 15 марта 2019 г.
 *      Author: fademike
 *      https://github.com/fademike
 */

#include "Kalman.h"


    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(struct Kalman * Kalman, double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
    	Kalman->rate = newRate - Kalman->bias;
    	Kalman->angle += dt * Kalman->rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
    	Kalman->P[0][0] += dt * (dt*Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    	Kalman->P[0][1] -= dt * Kalman->P[1][1];
    	Kalman->P[1][0] -= dt * Kalman->P[1][1];
    	Kalman->P[1][1] += Kalman->Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
    	Kalman->S = Kalman->P[0][0] + Kalman->R_measure;
        /* Step 5 */
    	Kalman->K[0] = Kalman->P[0][0] / Kalman->S;
    	Kalman->K[1] = Kalman->P[1][0] / Kalman->S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
    	Kalman->y = newAngle - Kalman->angle;
        /* Step 6 */
    	Kalman->angle += Kalman->K[0] * Kalman->y;
    	Kalman->bias += Kalman->K[1] * Kalman->y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
    	Kalman->P[0][0] -= Kalman->K[0] * Kalman->P[0][0];
    	Kalman->P[0][1] -= Kalman->K[0] * Kalman->P[0][1];
    	Kalman->P[1][0] -= Kalman->K[1] * Kalman->P[0][0];
    	Kalman->P[1][1] -= Kalman->K[1] * Kalman->P[0][1];

        return Kalman->angle;
    };





