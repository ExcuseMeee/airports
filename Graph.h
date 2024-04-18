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
  // EDGE IS DIRECTED BY DEFAULT. CONSIDERS ONLY DISTANCE BY DEFAULT
  void addEdge(const Vertex<T>& origin, const Vertex<T>& dest, int distance, int cost, bool directed = true, bool considerCost = false); 
  void print() const;

  void findShortestPath(const Vertex<T>& src, const Vertex<T>& dest); // print path, distance, and cost

  // create undirected graph using this graph as basis. graph chooses edges with least COST. created graph considers COST by default.
  Graph<T> createUndirected(); 
  void directConnections(); // display direct connections (to and from) each vertex

  void Prim_ShortestPath(const Vertex<T>& src); //prints prim path

  void kruskalMST(Graph<T>& initialGraph);
  void Kruskal_ShortestPath(const Vertex<T>& src); //prints kruskal path

  void setConsiderCost(bool considerCost); // set all edges to considerCost (set to true or false)


private:
  std::vector<Vertex<T>> vertices;
  std::vector<std::vector<Edge>> adjacencyLists;

  int getVertexIndex(const Vertex<T>& ver);
  void cleanVisited();

  bool areNeighbors(const Vertex<T>& src, const Vertex<T>& dest);

  

};

#endif
