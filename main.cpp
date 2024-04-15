#include <iostream>
#include <string>
#include "Vertex.h"
#include "Graph.h"
#include "Graph.cpp"

int main() {

  Vertex<std::string> a("A");
  Vertex<std::string> b("B");
  Vertex<std::string> c("C");

  Graph<std::string> g;

  g.insertVertex(a);
  g.insertVertex(b);
  g.insertVertex(c);

  g.addEdge(a, b, 10, 20, false);
  g.addEdge(b, c, 99, 99, false);

  g.print();

  return 0;
}