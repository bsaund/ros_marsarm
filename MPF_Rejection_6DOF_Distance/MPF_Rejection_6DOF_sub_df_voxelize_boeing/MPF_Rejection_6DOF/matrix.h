#ifndef MATRIX_H
#define MATRIX_H

void multiplyM(double matrixA[3][3], double matrixB[3][3], double resultM[3][3]);
void multiplyM(double matrixA[3][3], double matrixB[3], double resultM[3]);
void addM(double matrixA[3], double matrixB[3], double resultM[3]);
void subtractM(double matrixA[3], double matrixB[3], double resultM[3]);
void inverseMatrix(double matrix[3][3], double invMatrix[3][3]);
void rotationMatrix(particleFilter::cspace state, double rotationM[3][3]);

#endif // MATRIX_H