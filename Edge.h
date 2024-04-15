#ifndef EDGE_H
#define EDGE_H

// represents connection to another vertex. stores index of the neighbor vertex and distance and cost of connection. considers distance by default.
class Edge {
public:

  Edge(int source, int destination, int distance, int cost, bool considerCost = false) {
    this->distance = distance;
    this->cost = cost;
    this->source = source;
    this->destination = destination;
    this->considerCost = considerCost;
  }

  bool operator<(const Edge& other) const {
    if (this->considerCost == true)
      return this->cost < other.cost;
    else
      return this->distance < other.distance;
  }
  bool operator>(const Edge& other) const {
    if (this->considerCost == true)
      return this->cost > other.cost;
    else
      return this->distance > other.distance;
  }

  int distance;
  int cost;
  int source;
  int destination;
  bool considerCost;

};



#endif
