#ifndef RAYTRI_H
#define RAYTRI_H
#include <math.h>

#define EPSILON 0.000001
//#define CROSS(dest,v1,v2) \
//          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
//          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
//          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
//#define SUB(dest,v1,v2) \
//          dest[0]=v1[0]-v2[0]; \
//          dest[1]=v1[1]-v2[1]; \
//          dest[2]=v1[2]-v2[2]; 

int intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t, double *u, double *v)
{
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;
	/* find vectors for two edges sharing vert0 */
	SUB(edge1, vert1, vert0);
	SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = DOT(edge1, pvec);

	/* calculate distance from vert0 to ray origin */
	SUB(tvec, orig, vert0);
	inv_det = 1.0 / det;

	if (det > EPSILON)
	{
		/* calculate U parameter and test bounds */
		*u = DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* prepare to test V parameter */
		CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;

	}
	else if (det < -EPSILON)
	{
		/* calculate U parameter and test bounds */
		*u = DOT(tvec, pvec);
		if (*u > 0.0 || *u < det)
			return 0;

		/* prepare to test V parameter */
		CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	}
	else return 0;  /* ray is parallell to the plane of the triangle */

					/* calculate t, ray intersects triangle */
	*t = DOT(edge2, qvec) * inv_det;
	if (*t < 0)
		return 0;
	/*(*u) *= inv_det;
	(*v) *= inv_det;*/

	return 1;
}

#endif // RAYTRI_H