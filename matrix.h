/*
 * matrix.h
 *
 *  Created on: Dec 20, 2018
 *      Author: HANNAH
 */

#ifndef SRC_MAIN_MATRIX_H_
#define SRC_MAIN_MATRIX_H_

#define N 2
int m, n, i, j, k, a, b, c;

void getCofactor(float A[N][N], float temp[N][N], int p, int q, int n) {
	int i = 0, j = 0;
	for (int row = 0; row < n; row++) {
		for (int col = 0; col < n; col++) {
			if (row != p && col != q) {
				temp[i][j++] = A[row][col];
				if (j == n - 1) {
					j = 0;
					i++;
				}
			}
		}
	}
}

float determinant(float A[][N], int n) {
	float D = 0;
	if (n == 1)
		return A[0][0];
	float temp[N][N];
	int sign = 1;
	for (int f = 0; f < n; f++) {
		getCofactor(A, temp, 0, f, n);
		D += sign * A[0][f] * determinant(temp, n - 1);
		sign = -sign;
	}
	return D;
}

void adjoint(float A[][N], float adj[][N]) {
	if (N == 1) {
		adj[0][0] = 1;
		return;
	}
	int sign = 1;
	float temp[N][N];

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			getCofactor(A, temp, i, j, N);
			sign = ((i + j) % 2 == 0) ? 1 : -1;
			adj[j][i] = (sign) * (determinant(temp, N - 1));
		}
	}
}

void inverse(float A[][N], float inverse[][N]) {
	float det = determinant(A, N);
	float adj[N][N];
	adjoint(A, adj);
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			inverse[i][j] = adj[i][j] / (float) (det);

}

void transpose(float A[][n], float l[][m]) {
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			l[i][j] = A[j][i];

}

void add(float A[][n], float B[][n], float C[][n]) {
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[i][j] = A[i][j] + B[i][j];
}

void subtract(float A[][n], float B[][n], float C[][n]) {
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[i][j] = A[i][j] - B[i][j];
}

void multiply(float A[][b], float B[][c], float C[][c]) {
	for (int i = 0; i < a; i++)
		for (int j = 0; j < c; j++) {
			C[i][j] = 0;
			for (int k = 0; k < b; k++)
				C[i][j] += A[i][k] * B[k][j];
		}
}

#endif /* SRC_MAIN_MATRIX_H_ */
