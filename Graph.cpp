#include <iostream>
#include <vector>
#include <queue> // delete later
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
void Graph<T>::addEdge(const Vertex<T>& origin, const Vertex<T>& dest, int distance, int cost, bool directed, bool considerCost) {
  int origin_ind = getVertexIndex(origin);
  int dest_ind = getVertexIndex(dest);

  if (origin_ind == -1 || dest_ind == -1) throw std::string("[addEdge] invalid edges");

  adjacencyLists[origin_ind].push_back(Edge(origin_ind, dest_ind, distance, cost, considerCost));

  if (!directed && origin_ind != dest_ind)
    adjacencyLists[dest_ind].push_back(Edge(dest_ind, origin_ind, distance, cost, considerCost));

}

template<typename T>
void Graph<T>::print() const {
  std::cout << "[print]" << std::endl;
  for (int i = 0; i < vertices.size(); i++) {
    std::cout << this->vertices[i].getData() << ": ";
    for (Edge e : this->adjacencyLists[i]) {
      std::cout << "("
        << vertices[e.destination].getData() << ", "
        << "dist" << (e.considerCost ? "[]" : "[X]") << ": " << e.distance << ", "
        << "cost" << (e.considerCost ? "[X]" : "[]") << ": " << e.cost
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
  std::vector<int> distances(vertices.size(), 2147483647); // set all distances to infinite
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
        int totalCostToDest = costs[curIndex] + edge.cost;

        // update values if totalDistanceToDest is desirable (less than the current distance to reach destination)
        if (totalDistanceToDest < distances[edge.destination]) {
          distances[edge.destination] = totalDistanceToDest;
          costs[edge.destination] = totalCostToDest; // cost to reach destination
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

  std::cout << "[findShortestPath] ";
  // printing path and distance/cost
  if (distances[dest_ind] == 2147483647) {
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
void Graph<T>::Prim_ShortestPath(const Vertex<T>& src) {

  std::vector<int> parent(vertices.size(), -1); // store contructed MST
  std::vector<int> key(vertices.size(), 2147483647); // key values to pick min weight edge
  std::vector<bool> inside(vertices.size(), false); // checks if in MST

  //queue to store vertices
  MinHeap<Edge> minHeap;

  int src_ind = getVertexIndex(src);
  if (src_ind == -1) throw std::string("[Prim_ShortestPath] invalid vertices");
  key[src_ind] = 0;

  // inserts src into queue.
  minHeap.insert(Edge(src_ind, src_ind, 0, 0));

  while (!minHeap.empty()) {
    //extract min key value
    Edge minEdge = minHeap.popMin();
    int value = minEdge.source;

    //set as in the MST
    inside[value] = true;

    //update key and parent vertices with adjacent

  }

}


template<typename T>
void Graph<T>::kruskalMST(Graph<T>& initialGraph) {
  if (initialGraph.vertices.empty()) throw std::string("[kruskalMST] empty graph");
  cleanVisited();
  // sort edges from smallest to largest
  MinHeap<Edge> sortedEdges;
  // traverse graph in BFS and add edges
  int curInd = 0;
  // need a queue?
  std::queue<int> toVisit;
  toVisit.push(0);
  while (!toVisit.empty()) {
    int index = toVisit.front();
    if (vertices[index].getVisited() == false) {
      for (Edge edge : adjacencyLists[index]) {
        if (vertices[edge.destination].getVisited() == false) {
          sortedEdges.insert(edge);
          toVisit.push(edge.destination);

        }
      }
      vertices[index].setVisited(true);
    }
    toVisit.pop();
  }

  while (!sortedEdges.empty()) {
    Edge e = sortedEdges.popMin();
    std::cout << e.distance << " ";
  }
  // construct graph starting from smallest edge
  // ignore edges that connect already connected nodes
}

template<typename T>
void Graph<T>::Kruskal_ShortestPath(const Vertex<T>& src) {
  MinHeap<Edge> edgeHeap; //will store edges from least to greatest destination
  std::vector<Edge> Kruskal_MST; //will store necessary edges
  int max_edges = vertices.size() - 1; //max # of edges

  for (const std::vector<Edge>& adjList : adjacencyLists) { //adds edges to heap
    for (const Edge& edge : adjList) {
      edgeHeap.insert(edge);
    }
  }

  //some way to stop repeat edges (will implement later today)

  while (!edgeHeap.empty() && Kruskal_MST.size() < max_edges) {
    Edge edge = edgeHeap.popMin(); //add edge to mst
  }

  for (const Edge& edge : Kruskal_MST) {
    std::cout << vertices[edge.source].getData() << " - ";
    std::cout << vertices[edge.destination].getData() << " (";
    std::cout << edge.distance << ", ";
    std::cout << edge.cost << ")" << std::endl;
  }
}

template<typename T>
Graph<T> Graph<T>::createUndirected() {
  Graph<T> undirected;

  for (const Vertex<T>& v : this->vertices) {
    undirected.vertices.push_back(v);
    undirected.adjacencyLists.push_back(std::vector<Edge>());
  }

  // go through all connections
  for (int rowIndex = 0; rowIndex < this->adjacencyLists.size(); rowIndex++) {
    Vertex<T> srcVertex = this->vertices[rowIndex];

    // go through adj list of srcVertex
    for (const Edge& edge : this->adjacencyLists[rowIndex]) {
      Vertex<T> destVertex = this->vertices[edge.destination];
      if (undirected.areNeighbors(srcVertex, destVertex)) continue; // if already connected, move on

      // check if destVertex points back to srcVertex
      int minCost = edge.cost;
      int dist = edge.distance;
      for (const Edge& destVertex_edge : this->adjacencyLists[edge.destination]) {
        // destVertex points back to srcVertex
        if (destVertex_edge.destination == rowIndex) {
          // choose this connection if cost is less
          if (destVertex_edge.cost < minCost) {
            minCost = destVertex_edge.cost;
            dist = destVertex_edge.distance;
          }
          break;
        }
      }
      undirected.addEdge(srcVertex, destVertex, dist, minCost, false, true); // create undirected edge, consider cost
    }
  }

  return undirected;
}

template<typename T>
void Graph<T>::setConsiderCost(bool considerCost) {
  for (std::vector<Edge>& adjList : this->adjacencyLists) {
    for (Edge& edge : adjList) {
      edge.considerCost = considerCost;
    }
  }
}

template<typename T>
bool Graph<T>::areNeighbors(const Vertex<T>& src, const Vertex<T>& dest) {
  int srcIndex = getVertexIndex(src);
  int destIndex = getVertexIndex(dest);
  if (srcIndex == -1 || destIndex == -1) throw std::string("[areNeighbors] invalid vertices");

  // check src's adj list for dest
  for (const Edge& edge : this->adjacencyLists[srcIndex]) {
    if (edge.destination == destIndex) return true;
  }

  return false;
}
