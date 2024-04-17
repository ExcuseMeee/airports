#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "Vertex.h"
#include "Graph.h"
#include "Graph.cpp"

std::vector<std::string> tokenize(std::string& line, char delimiter = ' ') {
  // {srcAirport, destAirport, srcCity, srcState, destCity, destState, distance, cost}
  if (line.empty()) throw std::string("[tokenize] empty line");
  std::vector<std::string> tokens;

  std::istringstream iss(line);
  std::string token;
  while (std::getline(iss, token, delimiter)) {
    tokens.push_back(token);
    // std::cout << token << std::endl;
  }

  return tokens;

}

int main() {
  /*
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

  airports.addEdge(v, v2, 337, 0, false);
  airports.addEdge(v, v3, 1464, 0, false);
  airports.addEdge(v, v4, 1846, 0, false);
  airports.addEdge(v, v6, 2704, 0, false);
  airports.addEdge(v3, v4, 802, 0, false);
  airports.addEdge(v6, v4, 867, 0, false);
  airports.addEdge(v5, v6, 187, 0, false);
  airports.addEdge(v5, v7, 1090, 0, false);
  airports.addEdge(v6, v7, 1258, 0, false);
  airports.addEdge(v2, v7, 2342, 0, false);
  airports.addEdge(v2, v3, 1235, 0, false);
  airports.addEdge(v7, v3, 1121, 0, false);
  airports.addEdge(v4, v5, 740, 0, false);

  // airports.print();
  // airports.findShortestPath(v5, v);
  // airports.findShortestPath(v5, v2);
  // airports.findShortestPath(v5, v4);
  // airports.findShortestPath(v5, v3);

  airports.kruskalMST(airports);
  */

  /*
   Graph<std::string> directed;

   Vertex<std::string> A("A");
   Vertex<std::string> B("B");
   Vertex<std::string> C("C");
   Vertex<std::string> D("D");
   Vertex<std::string> E("E");

   directed.insertVertex(A);
   directed.insertVertex(B);
   directed.insertVertex(C);
   directed.insertVertex(D);
   directed.insertVertex(E);

   directed.addEdge(A, B, 0, 6, true);
   directed.addEdge(A, D, 0, 9, true);
   directed.addEdge(B, A, 0, 3, true);
   directed.addEdge(B, C, 0, 4, true);
   directed.addEdge(B, E, 0, 8, true);
   directed.addEdge(C, D, 0, 5, true);
   directed.addEdge(D, A, 0, 7, true);
   directed.addEdge(D, E, 0, 9, true);

   Graph<std::string> undirected = directed.createUndirected();
   undirected.setConsiderCost(true);
   undirected.print();
   */

   // testing to read CSV
  std::ifstream inFile("testing.csv");
  std::string line;
  while (std::getline(inFile, line)) {
    std::vector<std::string> tkns = tokenize(line, ',');
    for (auto t : tkns) std::cout << t << "|";
    std::cout << "\n";
  }


  return 0;
}