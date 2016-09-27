#ifndef MATRIX_H
#define MATRIX_H

void multiplyM(const double matrixA[3][3], const double matrixB[3][3], double resultM[3][3]);
void multiplyM(const double matrixA[3][3], const double matrixB[3], double resultM[3]);
void addM(const double matrixA[3], const double matrixB[3], double resultM[3]);
void subtractM(const double matrixA[3], const double matrixB[3], double resultM[3]);
void inverseMatrix(const double matrix[3][3], double invMatrix[3][3]);
void rotationMatrix(cspace state, double rotationM[3][3]);

#endif // MATRIX_H
