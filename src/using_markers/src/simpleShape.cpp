
#include "simpleShape.h"

Cube::Cube(){
  // m_numEdges = 12;
  
  edges[0] = Edge(0,0,0, 0,0,1);
  edges[1] = Edge(0,0,1, 0,1,1);
  edges[2] = Edge(0,1,1, 0,1,0);
  edges[3] = Edge(0,1,0, 0,0,0);
  
  edges[4] = Edge(0,0,0, 1,0,0);
  edges[5] = Edge(0,0,1, 1,0,1);
  edges[6] = Edge(0,1,0, 1,1,0);
  edges[7] = Edge(0,1,1, 1,1,1);

  edges[8] = Edge(1,0,0, 1,0,1);
  edges[9] = Edge(1,0,1, 1,1,1);
  edges[10] = Edge(1,1,1, 1,1,0);
  edges[11] = Edge(1,1,0, 1,0,0);

}

Edge Cube::getEdge(int edgeIndex) const{
  return edges[edgeIndex];
}

int Cube::numEdges() const{
  return m_numEdges;
}


Edge::Edge(double x1, double y1, double z1, double x2, double y2, double z2){
  point1.setX(x1);
  point1.setY(y1);
  point1.setZ(z1);
  point2.setX(x2);
  point2.setY(y2);
  point2.setZ(z2);
}

Edge::Edge(tf::Point p1, tf::Point p2){
  point1 = p1;
  point2 = p2;
}

Edge::Edge()
{
}





