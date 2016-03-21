#ifndef RAYTRI_H
#define RAYTRI_H

/* Ray-Triangle intersection algorithm */
/* intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t, double *u, double *v) */
/* return 1 if intersects, else return 0 */

int intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t, double *u, double *v);

#endif // RAYTRI_H