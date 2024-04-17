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

  Graph<T> createUndirected(); // create undirected graph using this graph as basis

  void Prim_ShortestPath(const Vertex<T>& src); //prints prim path

  void kruskalMST(Graph<T>& initialGraph);
  void Kruskal_ShortestPath(const Vertex<T>& src); //prints kruskal path


private:
  std::vector<Vertex<T>> vertices;
  std::vector<std::vector<Edge>> adjacencyLists;

  int getVertexIndex(const Vertex<T>& ver);
  void cleanVisited();

  

};

#endif
