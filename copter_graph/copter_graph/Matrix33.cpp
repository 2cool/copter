#include "stdafx.h"
#include "Matrix33.h"
#include "math.h"

float Matrix33D::determinant(const Matrix33D& a, const int k)
{
	Matrix33D b;
	float s = 1, det = 0;
	int i, j, m, n, c;
	if (k == 1) {
		return (a.m[0][0]);
	}
	else
	{
		det = 0;
		for (c = 0; c < k; c++) {
			m = 0;
			n = 0;
			for (i = 0; i < k; i++) {
				for (j = 0; j < k; j++) {
					b.m[i][j] = 0;
					if (i != 0 && j != c) {
						b.m[m][n] = a.m[i][j];
						if (n < (k - 2))
							n++;
						else {
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s * (a.m[0][c] * determinant(b, k - 1));
			s = -1 * s;
		}
	}
	return det;
}

Matrix33D Matrix33D::cofactor(const Matrix33D& num)
{
	Matrix33D fac;
	Matrix33D b;
	int p, q, m, n, i, j;
	for (q = 0; q < 3; q++)
	{
		for (p = 0; p < 3; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < 3; i++) {
				for (j = 0; j < 3; j++) {
					if (i != q && j != p) {
						b.m[m][n] = num.m[i][j];
						if (n < (3 - 2))
							n++;
						else {
							n = 0;
							m++;
						}
					}
				}
			}
			const float qp = q + p;
			fac.m[q][p] = pow(-1, qp) * determinant(b, 3 - 1);
		}
	}
	return fac;
}
Matrix33D Matrix33D::transpose() {
	return transpose(*this);
}
Matrix33D Matrix33D::transpose(const Matrix33D& a) {
	Matrix33D d;
	d.m[0][0] = a.m[0][0];
	d.m[1][0] = a.m[0][1];
	d.m[2][0] = a.m[0][2];
	d.m[0][1] = a.m[1][0];
	d.m[1][1] = a.m[1][1];
	d.m[2][1] = a.m[1][2];
	d.m[0][2] = a.m[2][0];
	d.m[1][2] = a.m[2][1];
	d.m[2][2] = a.m[2][2];
	return d;
}

Matrix33D Matrix33D::inverse() {
	return inverse(*this);
}
Matrix33D Matrix33D::inverse(const Matrix33D& num) {
	Matrix33D b = transpose(cofactor(num));
	Matrix33D inverse;
	float d = determinant(num, 3);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			inverse.m[i][j] = b.m[i][j] / d;

	return inverse;
}