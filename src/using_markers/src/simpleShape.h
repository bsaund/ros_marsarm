#ifndef SIMPLESHAPE_H
#define SIMPLESHAPE_H

#include <tf/transform_datatypes.h>

class Edge{
 public:
  Edge(double x1, double x2, double x3, double y1, double y2, double y3);
  Edge(tf::Point p1, tf::Point p2);
  Edge();
  tf::Point point1;
  tf::Point point2;
};


class Cube{
 public:
  int numEdges() const;
  Cube();
  Edge getEdge(int edgeIndex) const;
 private:
  static const int m_numEdges = 12;
  Edge edges[12];
};


#endif
