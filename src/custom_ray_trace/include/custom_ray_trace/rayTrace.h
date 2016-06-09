#ifndef RAYTRI_H
#define RAYTRI_H
using namespace std;
#include <array>
#include <vector>
typedef array<array<float, 3>, 4> vec4x3;



/* Ray-Triangle intersection algorithm */
/* intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t, double *u, double *v) */
/* return 1 if intersects, else return 0 */

/* int intersect_triangle(double orig[3], double dir[3], */
/* 	double vert0[3], double vert1[3], double vert2[3], */
/* 	double *t, double *u, double *v); */

/* int getIntersection(vector<vec4x3> &mesh, double pstart[3],  */
/* 		    double dir[3], double distToPart); */

/* Ray-Triangle intersection algorithm */
/* intersect_triangle(double orig[3], double dir[3],
   double vert0[3], double vert1[3], double vert2[3],
   double *t, double *u, double *v) */
/* return 1 if intersects, else return 0 */



#define EPSILON 0.000001
#define CROSS(dest,v1,v2)			\
  dest[0]=v1[1]*v2[2]-v1[2]*v2[1];		\
  dest[1]=v1[2]*v2[0]-v1[0]*v2[2];		\
  dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define SUB(dest,v1,v2)				\
  dest[0]=v1[0]-v2[0];				\
  dest[1]=v1[1]-v2[1];				\
  dest[2]=v1[2]-v2[2]; 

int intersect_triangle(double orig[3], double dir[3],
		       double vert0[3], double vert1[3], double vert2[3],
		       double &t, double &u, double &v)
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
      u = DOT(tvec, pvec);
      if (u < 0.0 || u > det)
	return 0;

      /* prepare to test V parameter */
      CROSS(qvec, tvec, edge1);

      /* calculate V parameter and test bounds */
      v = DOT(dir, qvec);
      if (v < 0.0 || u + v > det)
	return 0;

    }
  else if (det < -EPSILON)
    {
      /* calculate U parameter and test bounds */
      u = DOT(tvec, pvec);
      if (u > 0.0 || u < det)
	return 0;

      /* prepare to test V parameter */
      CROSS(qvec, tvec, edge1);

      /* calculate V parameter and test bounds */
      v = DOT(dir, qvec);
      if (v > 0.0 || u + v < det)
	return 0;
    }
  else return 0;  /* ray is parallell to the plane of the triangle */

  /* calculate t, ray intersects triangle */
  t = DOT(edge2, qvec) * inv_det;
  if (t < 0)
    return 0;
  /*(*u) *= inv_det;
    (*v) *= inv_det;*/

  return 1;
}


/*
 * Find the intersection point between a ray and meshes
 * Input: mesh: mesh arrays
 * 	      pstart: start point
 * 	      dir: ray direction
 * 	      intersection: intersection point
 * Output: 1 if intersect
 *         0 if not
 */
int getIntersection(vector<vec4x3> &mesh, double pstart[3], double dir[3], double &distToPart)
{
  int num_mesh = int(mesh.size());
  double vert0[3], vert1[3], vert2[3];
  double t;
  double u;
  double v;
  double tMin = 100000;
  for (int i = 0; i < num_mesh; i++)
    {
      vert0[0] = mesh[i][1][0];
      vert0[1] = mesh[i][1][1];
      vert0[2] = mesh[i][1][2];
      vert1[0] = mesh[i][2][0];
      vert1[1] = mesh[i][2][1];
      vert1[2] = mesh[i][2][2];
      vert2[0] = mesh[i][3][0];
      vert2[1] = mesh[i][3][1];
      vert2[2] = mesh[i][3][2];
      if (intersect_triangle(pstart, dir, vert0, vert1, vert2, t, u, v) == 1 && t < tMin)
	{
	  tMin = t;
	}

    }

  if (tMin == 100000)
    return 0;

  distToPart = tMin;
  // for (int i = 0; i < 3; i++)
  //   {
  //     intersection[i] = pstart[i] + dir[i] * tMin;
  //   }
	
  return 1;
}


#endif // RAYTRI_H
