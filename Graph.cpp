#include <iostream>
#include <vector>
#include "Graph.h"
#include "Vertex.h"

template<typename T>
Graph<T>::Graph() {}

template<typename T>
int Graph<T>::getVertexIndex(const Vertex<T>& ver) {
  for (int i = 0; i < this->vertices.size(); i++) {
    if (vertices[i].getData() == ver.getData()) return i;
  }
  return -1;
}

template<typename T>
void Graph<T>::insertVertex(const Vertex<T>& v) {
  if (getVertexIndex(v) != -1) return; // if vertex already exists, don't add it
  this->vertices.push_back(v);
  this->adjacencyLists.push_back(std::vector<Edge>());
}

template<typename T>
void Graph<T>::addEdge(const Vertex<T>& origin, const Vertex<T>& dest, int distance, int cost, bool directed) {
  int origin_ind = getVertexIndex(origin);
  int dest_ind = getVertexIndex(dest);

  if (origin_ind == -1 || dest_ind == -1) throw std::string("[addEdge] invalid edges");

  adjacencyLists[origin_ind].push_back(Edge(origin_ind, dest_ind, distance, cost));

  if (!directed && origin_ind != dest_ind)
    adjacencyLists[dest_ind].push_back(Edge(dest_ind, origin_ind, distance, cost));

}

template<typename T>
void Graph<T>::print() const {
  std::cout << "[print]" << std::endl;
  for (int i = 0; i < vertices.size(); i++) {
    std::cout << this->vertices[i].getData() << ": ";
    for (Edge e : this->adjacencyLists[i]) {
      std::cout << "("
        << vertices[e.destination].getData() << ", "
        << "dist: " << e.distance << ", "
        << "cost: " << e.cost
        << ") ";
    }
    std::cout << std::endl;
  }
}