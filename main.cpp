#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "Vertex.h"
#include "Graph.h"
#include "Graph.cpp"
#include "AirportData.h"
#include "UnionFind.h"
#include "UnionFind.cpp"
// should return {srcAirport, destAirport, srcCity, srcState, destCity, destState, distance, cost}
std::vector<std::string> tokenize(std::string& line, char delimiter = ' ');

void addAirportEdge(const std::vector<std::string>& tokens, Graph<AirportData>& airports);


int main() {

  // testing to read CSV
  Graph<AirportData> airports;
  std::ifstream inFile("testing.csv");
  std::string line;
  std::getline(inFile, line); // skip first line. ASSUMING first line is not data
  while (std::getline(inFile, line)) {
    try {
      std::vector<std::string> tkns = tokenize(line, ',');
      addAirportEdge(tkns, airports);
    }
    catch (const std::string& msg) {
      // if error occurs, skip this line
      std::cout << msg << "...skipping line\n";
      continue;
    }

  }
  try
  {

    // airports.print();
    airports.findShortestPath(Vertex<AirportData>(AirportData("A", "Src", "FL")), Vertex<AirportData>(AirportData("E", "Src", "FL")));
    // airports.directConnections();

    puts("");

    Graph<AirportData> undirected = airports.createUndirected(); // undirected graph considers COST by default
    // undirected.print();
    // undirected.findShortestPath(Vertex<AirportData>(AirportData("A", "Src", "FL")), Vertex<AirportData>(AirportData("E", "Src", "FL")));
    // undirected.kruskalMST();

    // Graph<AirportData> krusk = undirected.kruskalMST();
    // krusk.print();

    // undirected.Prim_ShortestPath();
    undirected.shortestPathsToState(Vertex<AirportData>(AirportData("A", "Src", "FL")), std::string("FL"));

  }
  catch (const std::string& e)
  {
    std::cout << e << '\n';
  }





  return 0;
}

std::vector<std::string> tokenize(std::string& line, char delimiter) {
  if (line.empty()) throw std::string("[tokenize] empty line");
  std::vector<std::string> tokens;

  std::istringstream iss(line);
  std::string token;
  while (std::getline(iss, token, delimiter)) {
    // strip token of leading/trailing whitespace and quotes
    if (token.front() == ' ' || token.front() == '"') token.erase(0, 1);
    if (token.back() == ' ' || token.back() == '"') token.erase(token.size() - 1, 1);

    if (!token.empty()) tokens.push_back(token);
  }

  // expecting to return vector of size 8. we can check this later
  return tokens;

}

// expects tokens = {srcAirport, destAirport, srcCity, srcState, destCity, destState, distance, cost}
// will create necessary vertices and add edge between them. edges are assumed directional
void addAirportEdge(const std::vector<std::string>& tokens, Graph<AirportData>& airports) {
  if (tokens.size() != 8) throw std::string("[addAirportEdge] incorrect tokens");
  AirportData srcAirport = AirportData(tokens.at(0), tokens.at(2), tokens.at(3));
  AirportData destAirport = AirportData(tokens.at(1), tokens.at(4), tokens.at(5));
  Vertex<AirportData> srcVertex(srcAirport);
  Vertex<AirportData> destVertex(destAirport);
  airports.insertVertex(srcVertex);
  airports.insertVertex(destVertex);
  airports.addEdge(srcVertex, destVertex, std::stoi(tokens.at(6)), std::stoi(tokens.at(7)));

}
