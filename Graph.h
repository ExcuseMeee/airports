#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <iostream>
#include "Vertex.h"
#include "Edge.h"

template<typename T>
class Graph {
public:
  Graph();

  void insertVertex(const Vertex<T>& v);
  void addEdge(const Vertex<T>& origin, const Vertex<T>& dest, int distance, int cost, bool directed = false);

  void print() const;

private:
  std::vector<Vertex<T>> vertices;
  std::vector<std::vector<Edge>> adjacencyLists;

  int getVertexIndex(const Vertex<T>& ver);

};


#endif