/*
 * kalman.h
 *
 *  Created on: Dec 20, 2018
 *      Author: HANNAH
 */
/*
 * kalman.h
 *
 *  Created on: Dec 20, 2018
 *      Author: HANNAH
 */

#ifndef SRC_MAIN_KALMAN_H_
#define SRC_MAIN_KALMAN_H_

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//my matrix library, you can use your own favorite matrix library
#include "matrix.h"
#define dt 10

int rowNumber;
float timeIndex,m_a1,c_a1,tau1;
float a1, a2, a3, a1d, a2d, a3d, a1dd, a2dd, a3dd;
float new_a1d, new_a2d, new_a3d;
float TIME = 0;
float A[2][2] = { { 1, dt }, { 0, 1 } };
float B[2][1] = { { dt * dt }, { dt } };
float H[2][2] = { { 1, 0 }, { 0, 1 } };
float I[2][2] = { 1,0,0,1};
float C[2][2] = { { 1, 0 }, { 0, 1 } };
float Q[2][2] = { { 1, 0 }, { 0, 1 } };
float R[2][2] = { { 1, 0 }, { 0, 1 } };
float L[2][2];
float y[2][1];
float K[2][2];
float x[2][1];
float state[2][1];
float action[1][1];
float lastState[2][1];
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float p[2][2] = { { 1, 0 }, { 0, 1 } };
float measurement[2][1];

void initKalman() {

	/*initializes the state*/
	state[0][0] = 0;
	state[1][0] = 0;

	lastState[0][0] = state[0][0];
	lastState[1][0] = state[1][0];

}

void kalman() {
	lastState[0][0] = state[0][0];
		lastState[1][0] = state[1][0];
	state[0][0] = c_a1;
	state[1][0] = a1d;

	measurement[0][0] = m_a1;
	measurement[1][0] = a1d;

	action[0][0] = tau1;

	float temp1[2][2];
	float temp2[2][2];
	float temp3[2][2];
	float temp4[2][2];
	float temp5[2][2];
	float temp6[2][2];
	float temp11[2][2];
	float temp12[2][2];
	float temp13[2][2];
	float temp7[2][1];
	float temp8[2][1];
	float temp9[2][1];
	float temp10[2][1];
	float temp14[2][1];
	float temp15[2][1];
	float temp16[2][1];
	float temp17[2][1];
	float temp18[2][1];
	float temp19[2][1];
	float temp20[2][2];
	float temp21[2][2];

	/************ Prediction Equations*****************/
	multiply(B, action, temp7);
	multiply(A, lastState, temp8);
	add(temp7, temp8, temp9);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 1; j++) {
			x[i][j] = temp9[i][j];
		}
	}
	float Z[2][2];
	transpose(A, L);
	multiply(P, L, temp1);
	multiply(A, temp1, temp2);
	add(Q, temp2, temp3);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			p[i][j] = temp3[i][j];
		}
	}
	transpose(H, Z);
	multiply(p, Z, temp4);
	multiply(H, temp4, temp5);
	add(temp5, R, temp6);
	inverse(temp6, temp11);
	multiply(H, temp11, temp12);
	multiply(p, temp12, temp13);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 1; j++) {
			K[i][j] = temp13[i][j];
		}
	}
	/************ Update Equations **********/
//K = multiply(p,multiply(C,add((multiply(C,multiply(p,transpose(C,Z))),R))));
	multiply(C, state, temp10);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 1; j++) {
			y[i][j] = temp10[i][j];
		}
	}
	multiply(C, lastState, temp15);
	subtract(y, temp15, temp16);
	multiply(K, temp16, temp17);
	add(x, temp17, temp18);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 1; j++) {
			state[i][j] = temp18[i][j];
		}
	}

	multiply(K, H, temp19);
	subtract(I, temp19, temp20);
	multiply(temp20, p, temp21);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 1; j++) {
			P[i][j] = temp21[i][j];
		}
	}

	a1 = state[0][0];
	a1d = state[1][0];

}

/* This function is not used since I am using position to get velocity (i.e. differentiation).
 * However I think that it is useful to include if you want velocity and position
 * from acceleration you would use it */
void integrate() {
	new_a1d = a1d + a1dd * dt;
	a1 += (new_a1d + a1d) * dt / 2;
	a1d = new_a1d;
	new_a2d = a2d + a2dd * dt;
	a2 += (new_a2d + a2d) * dt / 2;
	a2d = new_a2d;
	new_a3d = a3d + a3dd * dt;
	a3 += (new_a3d + a3d) * dt / 2;
	a3d = new_a3d;
	TIME += dt;
}
}
#endif
