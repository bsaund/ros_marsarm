#include "matrix.h"
#include <cmath>
// #include "particleFilter.h"

void multiplyM(const double matrixA[3][3], const double matrixB[3][3], double resultM[3][3])
{
  for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < 3; k++)
	{
	  resultM[j][k] = matrixA[j][0] * matrixB[0][k] + matrixA[j][1] * matrixB[1][k] + matrixA[j][2] * matrixB[2][k];
	}
    }
}

void multiplyM(const double matrixA[3][3], const double matrixB[3], double resultM[3])
{
  for (int j = 0; j < 3; j++)
    {
      resultM[j] = matrixA[j][0] * matrixB[0] + matrixA[j][1] * matrixB[1] + matrixA[j][2] * matrixB[2];
    }
}

void addM(const double matrixA[3], const double matrixB[3], double resultM[3])
{
  for (int i = 0; i < 3; i++)
    {
      resultM[i] = matrixA[i] + matrixB[i];
    }
}
void subtractM(const double matrixA[3], const double matrixB[3], double resultM[3])
{
  for (int i = 0; i < 3; i++)
    {
      resultM[i] = matrixA[i] - matrixB[i];
    }
}
void inverseMatrix(const double matrix[3][3], double invMatrix[3][3])
{
  double determinant = +matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
  double invdet = 1 / determinant;
  invMatrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2])*invdet;
  invMatrix[0][1] = -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1])*invdet;
  invMatrix[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1])*invdet;
  invMatrix[1][0] = -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])*invdet;
  invMatrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0])*invdet;
  invMatrix[1][2] = -(matrix[0][0] * matrix[1][2] - matrix[1][0] * matrix[0][2])*invdet;
  invMatrix[2][0] = (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1])*invdet;
  invMatrix[2][1] = -(matrix[0][0] * matrix[2][1] - matrix[2][0] * matrix[0][1])*invdet;
  invMatrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1])*invdet;
}
void rotationMatrix(cspace state, double rotationM[3][3])
{
  double rotationC[3][3] = { { cos(state[5]), -sin(state[5]), 0 },
			     { sin(state[5]), cos(state[5]), 0 },
			     { 0, 0, 1 } };
  double rotationB[3][3] = { { cos(state[4]), 0 , sin(state[4]) },
			     { 0, 1, 0 },
			     { -sin(state[4]), 0, cos(state[4]) } };
  double rotationA[3][3] = { { 1, 0, 0 },
			     { 0, cos(state[3]), -sin(state[3]) },
			     { 0, sin(state[3]), cos(state[3]) } };
  double tempRot[3][3];
  multiplyM(rotationC, rotationB, tempRot);
  multiplyM(tempRot, rotationA, rotationM);
}
