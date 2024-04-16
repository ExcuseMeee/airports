#include <iostream>
#include <string>
#include "Vertex.h"
#include "Graph.h"
#include "Graph.cpp"

int main() {

  Vertex<std::string> v("SFO");
  Vertex<std::string> v2("LAX");
  Vertex<std::string> v3("DFW");
  Vertex<std::string> v4("ORD");
  Vertex<std::string> v5("JFK");
  Vertex<std::string> v6("BOS");
  Vertex<std::string> v7("MIA");

  Graph<std::string> airports;

  airports.insertVertex(v);
  airports.insertVertex(v2);
  airports.insertVertex(v3);
  airports.insertVertex(v4);
  airports.insertVertex(v5);
  airports.insertVertex(v6);
  airports.insertVertex(v7);

  airports.addEdge(v, v2, 337, 0);
  airports.addEdge(v, v3, 1464, 0);
  airports.addEdge(v, v4, 1846, 0);
  airports.addEdge(v, v6, 2704, 0);
  airports.addEdge(v3, v4, 802, 0);
  airports.addEdge(v6, v4, 867, 0);
  airports.addEdge(v5, v6, 187, 0);
  airports.addEdge(v5, v7, 1090, 0);
  airports.addEdge(v6, v7, 1258, 0);
  airports.addEdge(v2, v7, 2342, 0);
  airports.addEdge(v2, v3, 1235, 0);
  airports.addEdge(v7, v3, 1121, 0);
  airports.addEdge(v4, v5, 740, 0);

  // airports.print();
  airports.findShortestPath(v5, v);
  airports.findShortestPath(v5, v2);
  airports.findShortestPath(v5, v4);
  airports.findShortestPath(v5, v3);


  return 0;
}