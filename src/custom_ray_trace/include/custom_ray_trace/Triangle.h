#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <cmath>
#include <array>
#include "rayTrace.h"
#include "Object.h"
#include <iostream>

struct Triangle : public Object {
	typedef array<array<double, 3>, 4> vec4x3;
	vec4x3 vertices;
	Vector3 center;
	Triangle(vec4x3 v): vertices(v), 
	center((vertices[1][0] + vertices[1][1] + vertices[1][2]) / 3.0, 
		   (vertices[2][0] + vertices[2][1] + vertices[2][2]) / 3.0, 
		   (vertices[3][0] + vertices[3][1] + vertices[3][2]) / 3.0) { }
	bool getIntersection(const BVHRay& ray, IntersectionInfo* I) const {
		array<double, 3> pstart = { ray.o.x, ray.o.y, ray.o.z };
		array<double, 3> dir = { ray.d.x, ray.d.y, ray.d.z };
		double t, u, v;
		if (!intersect_triangle(pstart, dir, &vertices[1][0], &vertices[2][0], &vertices[3][0], t, u, v))
			return false;
		I->object = this;
		I->t = t;
		return true;
	}

	Vector3 getNormal(const IntersectionInfo& I) const {
		Vector3 norm (vertices[0][0], vertices[0][1], vertices[0][2]);
		return normalize(norm);
	}

	BBox getBBox() const {
		double xmin = min(vertices[1][0], min(vertices[2][0], vertices[3][0]));
		double xmax = max(vertices[1][0], max(vertices[2][0], vertices[3][0]));
		double ymin = min(vertices[1][1], min(vertices[2][1], vertices[3][1]));
		double ymax = max(vertices[1][1], max(vertices[2][1], vertices[3][1]));
		double zmin = min(vertices[1][2], min(vertices[2][2], vertices[3][2]));
		double zmax = max(vertices[1][2], max(vertices[2][2], vertices[3][2]));
		return BBox(Vector3(xmin, ymin, zmin), Vector3(xmax, ymax, zmax));
	}

	Vector3 getCentroid() const {
		return center;
	}

};

#endif // TRIANGLE_H
