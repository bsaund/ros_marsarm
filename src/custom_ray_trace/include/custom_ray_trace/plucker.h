#ifndef PLUCKER_H
#define PLUCKER_H
#include <array>
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define CROSS(dest,v1,v2)			\
  dest[0]=v1[1]*v2[2]-v1[2]*v2[1];		\
  dest[1]=v1[2]*v2[0]-v1[0]*v2[2];		\
  dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define SUB(dest,v1,v2)				\
  dest[0]=v1[0]-v2[0];				\
  dest[1]=v1[1]-v2[1];				\
  dest[2]=v1[2]-v2[2]; 

using namespace std;

struct plucker {
  array<float,3> U;
  array<float,3> V;
};

typedef array<plucker,3> pluckerTriangle;

bool pluckerSide(const plucker &p1, const plucker &p2){
  return (DOT(p1.U, p2.V) + DOT(p2.U, p1.V)) > 0;
}

plucker pointsToPluckerRay(array<float,3> p1, array<float,3> p2)
{
  plucker pluck;
  SUB(pluck.U, p2, p1);
  CROSS(pluck.V, p2, p1);
  return pluck;
}

plucker pointDirToPluckerRay(array<float,3> p1, array<float,3> d1)
{
  plucker pluck;
  pluck.U = d1;
  CROSS(pluck.V, d1, p1);
  return pluck;
}

plucker pointDirToPluckerRay(array<double,3> p1, array<double,3> d1)
{
  plucker pluck;
  pluck.U[0] = d1[0];
  pluck.U[1] = d1[1];
  pluck.U[2] = d1[2];
  CROSS(pluck.V, d1, p1);
  return pluck;
}

pluckerTriangle pointsToPluckerTri(array<float,3> p1, array<float,3> p2, array<float,3> p3)
{
  pluckerTriangle pTri;
  pTri[0] = pointsToPluckerRay(p1, p2);
  pTri[1] = pointsToPluckerRay(p2, p3);
  pTri[2] = pointsToPluckerRay(p3, p1);
  return pTri;
}


#endif //PLUCKER_H
