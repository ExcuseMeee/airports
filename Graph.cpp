#include <iostream>
#include <vector>
#include "Graph.h"
#include "Vertex.h"
#include "MinHeap.h"
#include "MinHeap.cpp"
#include "Stack.h"
#include "Stack.cpp"
#include "UnionFind.h"
#include "AirportData.h"


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

  //use dijkstra
  cleanVisited();
  std::vector<int> distances(vertices.size(), 2147483647); //set all distances to infinite
  distances[src_ind] = 0; //distance to self is 0

  std::vector<int> costs(vertices.size()); //cost to reach each node
  std::vector<int> paths(vertices.size());

  MinHeap<Edge> minHeap;
  int visited = 0;
  int curIndex = src_ind;
  while (visited < vertices.size()) {
    for (Edge edge : adjacencyLists[curIndex]) {
      //only care about unvisited neighbors. skip visited neighbors
      if (vertices[edge.destination].getVisited() == false) {
        minHeap.insert(edge);
        int totalDistanceToDest = distances[curIndex] + edge.distance;
        int totalCostToDest = costs[curIndex] + edge.cost;

        //update values if totalDistanceToDest is desirable (less than the current distance to reach destination)
        if (totalDistanceToDest < distances[edge.destination]) {
          distances[edge.destination] = totalDistanceToDest;
          costs[edge.destination] = totalCostToDest; //cost to reach destination
          paths[edge.destination] = curIndex; //we reached destination FROM curIndex
        }

      }
    }

    //once we go through all neighbors, we mark this vertex visited, then choose next smallest edge
    vertices[curIndex].setVisited(true);
    if (!minHeap.empty()) {
      Edge smallestEdge = minHeap.popMin();
      curIndex = smallestEdge.destination; //move to vertex of smallest edge
    }
    visited++;
  }

  cleanVisited();

  std::cout << "Shortest path " << src.getData() << " to " << dest.getData() << ": ";
  //printing path and distance/cost
  if (distances[dest_ind] == 2147483647) {
    std::cout << "Unreachable" << std::endl;
  }
  else {
    //use stack to print paths. paths maps from destination to source, we want source to destination
    Stack<int> st;
    int ind = dest_ind;
    st.push(ind);
    do {
      st.push(paths[ind]);
      ind = paths[ind];
    } while (ind != src_ind);

    while (!st.empty()) {
      int i = st.top();
      std::cout << vertices[i].getData();
      st.pop();
      if (!st.empty()) std::cout << "->";
    }

    std::cout << " (Distance: " << distances.at(dest_ind) << ", " << "Cost: " << costs.at(dest_ind) << ")" << std::endl;

  }


}

template<typename T>
void Graph<T>::shortestPathsToState(const Vertex<T>& origin, std::string dest_state) {
  //find all destination airports for the state
  bool pathPresent = false;
  std::cout << "Shortest Paths to " << dest_state << " airports from " << origin.getData() << std::endl;
  for (const Vertex<T>& vertex : vertices) { //adds vertices to graph
    AirportData vertexAirportData = vertex.getData();
    if (dest_state == vertexAirportData.stateCode) {
      findShortestPath(origin, vertex); // NOTE: shortest path depends on whether the edge considers cost or distance
      pathPresent = true;
    }
  }
  if (pathPresent == false) {
    std::cout << "[shortestPathstoState] There are no valid paths" << std::endl;
  }
}

template<typename T>
void Graph<T>::shortestPathsWithStops(const Vertex<T>& origin, const Vertex<T>& dest, int stops) {
  std::cout << "[Shortest Paths with " << stops << " Stops] " << origin.getData() << " to " << dest.getData() << std::endl;
  // Find the index of the destination vertex
  int destIndex = getVertexIndex(dest);

  // If the destination vertex doesn't exist, return
  if (destIndex == -1) {
    std::cout << "[SPWS]Destination vertex not found" << std::endl;
    return;
  }

  // Perform DFS starting from the origin vertex
  Stack<std::vector<int>> dfsStack;
  std::vector<int> initialPath = { getVertexIndex(origin) };
  dfsStack.push(initialPath);
  int shortestDistance = 2147483647;
  int shortestCost = 0;
  std::vector<int> shortestPath;


  while (!dfsStack.empty()) {
    std::vector<int> currentPath = dfsStack.top();
    dfsStack.pop();

    int currentVertexIndex = currentPath.back();
    if (currentPath.size() - 2 == stops && currentVertexIndex == destIndex) {
      // Found a path with the specified number of stops
      int totalDistance = 0;
      int totalCost = 0;
      for (int i = 0; i < currentPath.size() - 1; ++i) {
        int srcIndex = currentPath[i];
        int destIndex = currentPath[i + 1];
        for (const Edge& edge : adjacencyLists[srcIndex]) {
          if (edge.destination == destIndex) {
            totalDistance += edge.distance;
            totalCost += edge.cost;
            break;
          }
        }
      }
      if (totalDistance < shortestDistance) {
        shortestDistance = totalDistance;
        shortestCost = totalCost;
        shortestPath = currentPath;
      }
    }

    // If num of stop > stops needed, skip
    if (currentPath.size() - 1 > stops) {
      continue;
    }

    // Explore neighbors
    for (const Edge& edge : adjacencyLists[currentVertexIndex]) {
      std::vector<int> newPath = currentPath;
      newPath.push_back(edge.destination);
      dfsStack.push(newPath);
    }

  }
  for (int i = 0; i < shortestPath.size() - 1; ++i) {
    std::cout << vertices[shortestPath[i]].getData() << " -> ";
  }
  std::cout << vertices[shortestPath.back()].getData();
  std::cout << " (Total Distance: " << shortestDistance << ", Total Cost: " << shortestCost << ")" << std::endl;
}

template<typename T>
void Graph<T>::cleanVisited() {
  for (Vertex<T>& v : this->vertices) {
    v.setVisited(false);
  }
}


template<typename T>
Graph<T> Graph<T>::Prim_ShortestPath() {
    std::vector<int> key(vertices.size(), 2147483647); //key values to pick min weight edge
    std::vector<bool> inside(vertices.size(), false); //checks if in MST

    MinHeap<Edge> minHeap;
    int total_edges = 0;
    int total_cost = 0;

    key[0] = 0;
    minHeap.insert(Edge(0, 0, 0, 0, true)); //insert the starting vertex

    Graph<T> prim_graph; //create a new graph to represent the MST

    while (total_edges < vertices.size() && !minHeap.empty()) {
        Edge minEdge = minHeap.popMin();
        int minEdgeSource = minEdge.source;
        int minEdgeDestination = minEdge.destination;

        if (!inside[minEdgeSource] || !inside[minEdgeDestination]) {
            inside[minEdgeSource] = true;
            inside[minEdgeDestination] = true;
            total_edges++;
            total_cost += minEdge.cost;

            prim_graph.insertVertex(vertices[minEdgeSource]);
            prim_graph.insertVertex(vertices[minEdgeDestination]);
            prim_graph.addEdge(vertices[minEdgeSource], vertices[minEdgeDestination], minEdge.distance, minEdge.cost, false, minEdge.considerCost);

            std::cout << "[Prim's MST] " << vertices[minEdgeSource].getData() << " - "
                      << vertices[minEdgeDestination].getData() << " ("
                      << minEdge.distance << ", "
                      << minEdge.cost << ")" << std::endl;

            for (const Edge& edge : adjacencyLists[minEdgeDestination]) {
                int v = edge.destination;
                if (!inside[v] && edge.cost < key[v]) {
                    key[v] = edge.cost;
                    minHeap.insert(edge);
                }
            }
        }
    }
    
    std::cout << "[Prim's MST] Total Edge Cost: " << total_cost << std::endl;

    return prim_graph;
}

template<typename T>
Graph<T> Graph<T>::kruskalMST() {
  MinHeap<Edge> edgeHeap; //will store edges from least to greatest destination
  std::vector<Edge> MST_final_edges; //will store the final edges
  Graph<T> Kruskal_MST_Graph; //will store MST
  int max_edges = vertices.size() - 1; //max # of edges
  int total_edge_cost = 0;
  int total_edges = 0;

  for (const std::vector<Edge>& adjList : adjacencyLists) { //adds edges to heap
    for (const Edge& edge : adjList) {
      edgeHeap.insert(edge);
    }
  }

  for (const Vertex<T>& vertex : vertices) { //adds vertices to graph
    Kruskal_MST_Graph.insertVertex(vertex);
  }

  UnionFind uf(vertices.size());

  while (!edgeHeap.empty() && total_edges < max_edges) {
    Edge edge = edgeHeap.popMin(); //add edge to MST

    if (uf.find(edge.source) != uf.find(edge.destination)) { //check for cycles
      uf.unionFunction(edge.source, edge.destination);
      MST_final_edges.push_back(edge);
      Kruskal_MST_Graph.addEdge(vertices[edge.source], vertices[edge.destination], edge.distance, edge.cost, false, edge.considerCost);
      total_edge_cost += edge.cost;
      total_edges++;
    }
  }
  for (const Edge& edge : MST_final_edges) {
    std::cout << "[Kruskal MST] ";
    std::cout << vertices[edge.source].getData() << " - ";
    std::cout << vertices[edge.destination].getData() << " (";
    std::cout << edge.distance << ", ";
    std::cout << edge.cost << ")" << std::endl;
  }

  std::cout << "[Kruskal MST] Total Edge Cost " << total_edge_cost << std::endl;

  return Kruskal_MST_Graph;
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

template<typename T>
void Graph<T>::directConnections() {
  std::cout << "[directConnections] ";
  std::vector<int> connections(this->vertices.size()); // stores # of connections for each vertex

  // loop through entire adjList
  for (int srcIndex = 0; srcIndex < this->adjacencyLists.size(); srcIndex++) {
    connections[srcIndex] += adjacencyLists[srcIndex].size(); // size of adj list is how many outbound connections you have
    for (Edge edge : adjacencyLists[srcIndex]) {
      int neighborIndex = edge.destination; // inbound connections. increment the destination index
      connections[neighborIndex]++;
    }
  }

  // all this to print in descending....
  MinHeap<Edge> minHeap;

  for (int i = 0; i < connections.size(); i++) {
    Edge fakeEdge = Edge(i, -1, connections[i], -1, false); // using Edge as a key value pair
    minHeap.insert(fakeEdge);
  }

  Stack<Edge> s;

  while (!minHeap.empty()) {
    s.push(minHeap.popMin());
  }

  while (!s.empty()) {
    Edge fakeEdge = s.top();
    std::cout << vertices[fakeEdge.source].getData();
    std::cout << ": " << fakeEdge.distance << ", ";
    s.pop();
  }
  std::cout << std::endl;


}
