#pragma once
#include "stdafx.h"
#include <iostream>
#include <string>

class Matrix33D
{
private:


public:
	float m[3][3];

	friend Matrix33D operator + (const Matrix33D& m0, const double v[]) {

	}

	friend Matrix33D operator + (const Matrix33D& m0, const Matrix33D& m1)
	{
		Matrix33D d;
		d.m[0][0] = m0.m[0][0] + m1.m[0][0];
		d.m[1][0] = m0.m[1][0] + m1.m[1][0];
		d.m[2][0] = m0.m[2][0] + m1.m[2][0];
		d.m[0][1] = m0.m[0][1] + m1.m[0][1];
		d.m[1][1] = m0.m[1][1] + m1.m[1][1];
		d.m[2][1] = m0.m[2][1] + m1.m[2][1];
		d.m[0][2] = m0.m[0][2] + m1.m[0][2];
		d.m[1][2] = m0.m[1][2] + m1.m[1][2];
		d.m[2][2] = m0.m[2][2] + m1.m[2][2];

		return d;
	}

	friend Matrix33D operator - (const Matrix33D& m0, const Matrix33D& m1)
	{
		Matrix33D d;
		d.m[0][0] = m0.m[0][0] - m1.m[0][0];
		d.m[1][0] = m0.m[1][0] - m1.m[1][0];
		d.m[2][0] = m0.m[2][0] - m1.m[2][0];
		d.m[0][1] = m0.m[0][1] - m1.m[0][1];
		d.m[1][1] = m0.m[1][1] - m1.m[1][1];
		d.m[2][1] = m0.m[2][1] - m1.m[2][1];
		d.m[0][2] = m0.m[0][2] - m1.m[0][2];
		d.m[1][2] = m0.m[1][2] - m1.m[1][2];
		d.m[2][2] = m0.m[2][2] - m1.m[2][2];

		return d;
	}
	friend Matrix33D operator * (const Matrix33D& m0, const Matrix33D& m1)
	{
		Matrix33D d;
		d.m[0][0] = m0.m[0][0] * m1.m[0][0] + m0.m[0][1] * m1.m[1][0] + m0.m[0][2] * m1.m[2][0];
		d.m[1][0] = m0.m[1][0] * m1.m[0][0] + m0.m[1][1] * m1.m[1][0] + m0.m[1][2] * m1.m[2][0];
		d.m[2][0] = m0.m[2][0] * m1.m[0][0] + m0.m[2][1] * m1.m[1][0] + m0.m[2][2] * m1.m[2][0];
		d.m[0][1] = m0.m[0][0] * m1.m[0][1] + m0.m[0][1] * m1.m[1][1] + m0.m[0][2] * m1.m[2][1];
		d.m[1][1] = m0.m[1][0] * m1.m[0][1] + m0.m[1][1] * m1.m[1][1] + m0.m[1][2] * m1.m[2][1];
		d.m[2][1] = m0.m[2][0] * m1.m[0][1] + m0.m[2][1] * m1.m[1][1] + m0.m[2][2] * m1.m[2][1];
		d.m[0][2] = m0.m[0][0] * m1.m[0][2] + m0.m[0][1] * m1.m[1][2] + m0.m[0][2] * m1.m[2][2];
		d.m[1][2] = m0.m[1][0] * m1.m[0][2] + m0.m[1][1] * m1.m[1][2] + m0.m[1][2] * m1.m[2][2];
		d.m[2][2] = m0.m[2][0] * m1.m[0][2] + m0.m[2][1] * m1.m[1][2] + m0.m[2][2] * m1.m[2][2];

		return d;
	}


	float determinant(const Matrix33D& m, const int k);
	Matrix33D cofactor(const Matrix33D& m);
	Matrix33D inverse(const Matrix33D& m);
	Matrix33D transpose(const Matrix33D& m);
	Matrix33D transpose();
	Matrix33D inverse();
};