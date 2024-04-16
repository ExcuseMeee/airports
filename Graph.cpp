#include <iostream>
#include <vector>
#include "Graph.h"
#include "Vertex.h"
#include "MinHeap.h"
#include "MinHeap.cpp"
#include "Stack.h"
#include "Stack.cpp"

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

template<typename T>
void Graph<T>::findShortestPath(const Vertex<T>& src, const Vertex<T>& dest) {
  int src_ind = getVertexIndex(src);
  int dest_ind = getVertexIndex(dest);

  if (src_ind == -1 || dest_ind == -1) throw std::string("[findShortestPath] invalid vertices");

  // use dijkstra
  cleanVisited();
  std::vector<int> distances(vertices.size(), INT_MAX); // set all distances to infinite
  distances[src_ind] = 0; // distance to self is 0

  std::vector<int> costs(vertices.size()); // cost to reach each node
  std::vector<int> paths(vertices.size());

  MinHeap<Edge> minHeap;
  int visited = 0;
  int curIndex = src_ind;
  while (visited < vertices.size()) {
    for (Edge edge : adjacencyLists[curIndex]) {
      // only care about unvisited neighbors. skip visited neighbors
      if (vertices[edge.destination].getVisited() == false) {
        minHeap.insert(edge);
        int totalDistanceToDest = distances[curIndex] + edge.distance;

        // update values if totalDistanceToDest is desirable (less than the current distance to reach destination)
        if (totalDistanceToDest < distances[edge.destination]) {
          distances[edge.destination] = totalDistanceToDest;
          costs[edge.destination] = edge.cost; // cost to reach destination
          paths[edge.destination] = curIndex; // we reached destination FROM curIndex
        }
      }
    }

    // once we go through all neighbors, we mark this vertex visited, then choose next smallest edge
    if (vertices[curIndex].getVisited() == false) visited++; // prevent incrementing if vertex was already visited (this can probably happen)
    vertices[curIndex].setVisited(true);
    Edge smallestEdge = minHeap.popMin();
    curIndex = smallestEdge.destination; // move to vertex of smallest edge
  }

  cleanVisited();

  // printing path and distance/cost
  if (distances[dest_ind] == INT_MAX) {
    std::cout << "Unreachable" << std::endl;
  }
  else {
    // use stack to print paths. paths maps from destination to source, we want source to destination
    Stack<int> st;
    int ind = dest_ind;
    st.push(ind);
    do {
      st.push(paths[ind]);
      ind = paths[ind];
    } while (ind != src_ind);

    while (!st.empty()) {
      int i = st.top();
      std::cout << vertices[i].getData() << "->";
      st.pop();
    }
    std::cout << "(end) ";


    std::cout << "Distance: " << distances.at(dest_ind) << ", " << "Cost: " << costs.at(dest_ind) << std::endl;

  }



}

template<typename T>
void Graph<T>::cleanVisited() {
  for (Vertex<T>& v : this->vertices) {
    v.setVisited(false);
  }
}

template<typename T>
void Graph<T>::Prim_ShortestPath(const Vertex<T>& src){
  
  std::vector<int> parent(vertices.size(), -1); // store contructed MST
  std::vector<int> key(vertices.size(), INT_MAX); // key values to pick min weight edge
  std::vector<bool> inside(vertices.size(), false); // checks if in MST
  
  //queue to store vertices
  MinHeap<Edge> minHeap;

  int src_ind = getVertexIndex(src);
  if (src_ind == -1) throw std::string("[Prim_ShortestPath] invalid vertices");
  key[src_ind] = 0;

  // inserts src into queue.
  minHeap.insert(Edge(src_ind, src_ind, 0, 0));

  while(!minHeap.empty()){
    //extract min key value
    Edge minEdge = minHeap.popMin();
    int value = minEdge.source;

    //set as in the MST
    inside[value] = true;

    //update key and parent vertices with adjacent
    
  }

}
