/*
 * Kalman.h
 *
 *  Created on: 15 марта 2019 г.
 *      Author: fademike
 *      GIT: https://github.com/fademike
 */

#ifndef KALMAN_H_
#define KALMAN_H_



struct Kalman {
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 matrix
    double y; // Angle difference - 1x1 matrix
    double S; // Estimate error - 1x1 matrix

};

//struct Kalman Kalman1 = {0.001, 0.003, 0.03, 0, 0, 0, {{0,0},{0,0}}, {0,0}, 0, 0}, Kalman2 = {0.001, 0.003, 0.03, 0, 0, 0, {{0,0},{0,0}}, {0,0}, 0, 0};

double getAngle(struct Kalman * Kalman, double newAngle, double newRate, double dt);


#endif /* KALMAN_H_ */
