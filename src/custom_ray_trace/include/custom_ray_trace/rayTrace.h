#ifndef RAYTRI_H
#define RAYTRI_H
using namespace std;
#include <array>
#include <vector>
#include "plucker.h"
#include "stlParser.h"
#include <iostream>

  



/* Ray-Triangle intersection algorithm */
/* intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t, double *u, double *v) */
/* return 1 if intersects, else return 0 */

/* int intersect_triangle(double orig[3], double dir[3], */
/* 	double vert0[3], double vert1[3], double vert2[3], */
/* 	double *t, double *u, double *v); */

/* int getIntersection(stlMesh &mesh, double pstart[3],  */
/* 		    double dir[3], double distToPart); */

/* Ray-Triangle intersection algorithm */
/* intersect_triangle(double orig[3], double dir[3],
   double vert0[3], double vert1[3], double vert2[3],
   double *t, double *u, double *v) */
/* return 1 if intersects, else return 0 */



#define EPSILON 0.000001
int intersect_triangle(array<double,3> orig, array<double,3> dir,
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
  inv_det = 1.0 / det;
  t = DOT(edge2, qvec) * inv_det;
  if (t < 0)
    return 0;
  /*(*u) *= inv_det;
    (*v) *= inv_det;*/

  return 1;
}


/*
 * This uses the triangle ray intersect algorithm in Plucker coordinates
 *  returns true if the ray intersects the triangle
 *  this is done by determining if the ray passes each edge of the triangle 
 *  on the same side (counter-clockwise or clockwise)
 */
bool triangleRayIntersectPlucker(const plucker &ray, const pluckerTriangle &pTri)
{
  bool side[3] = {pluckerSide(ray, pTri[0]),
  		  pluckerSide(ray, pTri[1]),
  		  pluckerSide(ray, pTri[2])};

  /* bool side[3] = {(DOT(ray.U, pTri[0].V) + DOT(pTri[0].U, ray.V)) > 0, */
  /* 		  (DOT(ray.U, pTri[0].V) + DOT(pTri[0].U, ray.V)) > 0, */
  /* 		  (DOT(ray.U, pTri[0].V) + DOT(pTri[0].U, ray.V)) > 0}; */
  /* bool side[3] = {(DOT(ray.U, ray.V) + DOT(ray.U, ray.V)) > 0, */
  /* 		  (DOT(ray.U, ray.V) + DOT(ray.U, ray.V)) > 0, */
  /* 		  (DOT(ray.U, ray.V) + DOT(ray.U, ray.V)) > 0}; */
  /* bool side[3] = {false, false, false}; */


  if(side[0] && side[1] && side[2])
    return true;
  if(!side[0] && !side[1] && !side[2])
    return true;
  return false;
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
int getIntersection(pluckerMesh &pMesh, array<double,3> pstart, 
		    array<double,3> dir, double &distToPart)
{
  stlMesh mesh = pMesh.stl;
  /* cout << "starting getIntersection" << endl; */
  /* cout << "stl: " << endl; */
  /* for(int i=0; i<3; i++){ */
  /*   for(int j=0; j<3; j++){ */
  /*     cout << mesh[0][i+1][j] << ", "; */
  /*   } */
  /*   cout << endl; */
  /* } */

  /* cout << endl << "plucker: " << endl; */
  /* for(int i=0; i<3; i++){ */
  /*   cout << "U:" << endl; */
  /*   for(int j=0; j<3; j++){ */
  /*     cout << pMesh.plucker[0][i].U[j] << ", "; */
  /*   } */
  /*   cout << endl << "V: " <<endl; */
  /*   for(int j=0; j<3; j++){ */
  /*     cout << pMesh.plucker[0][i].V[j] << ", "; */
  /*   } */
  /*   cout << endl; */
  /* } */

  /* plucker pluckerRay = pointDirToPluckerRay(pstart, dir); */

  int num_mesh = int(mesh.size());
  double vert0[3], vert1[3], vert2[3];
  double t;
  double u;
  double v;
  double tMin = 100000;
  /* cout << "pMesh size: " << pMesh.plucker.size() << "stlMesh size: " << num_mesh << endl; */

  for (int i = 0; i < num_mesh; i++) {
    /* bool pluckerIntersect = triangleRayIntersectPlucker(pluckerRay, pMesh.plucker[i]); */
    /* if(!pluckerIntersect) */
    /*   continue; */

    /* continue; */

    vert0[0] = mesh[i][1][0];
    vert0[1] = mesh[i][1][1];
    vert0[2] = mesh[i][1][2];
    vert1[0] = mesh[i][2][0];
    vert1[1] = mesh[i][2][1];
    vert1[2] = mesh[i][2][2];
    vert2[0] = mesh[i][3][0];
    vert2[1] = mesh[i][3][1];
    vert2[2] = mesh[i][3][2];
    /* if (intersect_triangle(pstart, dir, vert0, vert1, vert2, t, u, v) == 1 && t < tMin) */
    /* 	{ */
    /* 	  tMin = t; */
    /* 	} */
    bool stlIntersect = intersect_triangle(pstart, dir, vert0, vert1, vert2, t, u, v) == 1;
    if (stlIntersect && t < tMin){
      tMin = t;
    }
      
    /* if(stlIntersect != pluckerIntersect){ */
    /*   cout << "methods disagree" << endl; */
    /*   cout << "Plucker " << pluckerIntersect << "   stl " << stlIntersect << endl; */
    /* } */

    
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
