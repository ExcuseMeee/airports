#ifndef GRAPH_H
#define GRAPH_H
#include "Vertex.h"
#include "Edge.h"
#include <vector>


template <typename T>
class Graph {
public:
    Graph() {}

    void insert_vertex(const Vertex<T>& ver);
    void add_edge(const Vertex<T>& ver1, const Vertex<T>& ver2, int weight); //connect ver1 with ver2

    void print() const;

    void DFS(Vertex<T>& ver);
    void BFS(Vertex<T>& ver);

    int shortest_path(const Vertex<T>& src, const Vertex<T>& dest);
    int dijkstra_shortest_path(const Vertex<T>& src, const Vertex<T>& dest);
private:
    std::vector<Vertex<T>> vertices; //nodes
    std::vector<std::vector<Edge>> edges; //connections

    void clean_visited();

    void DFS_helper(Vertex<T>& ver);
    int get_vertex_index(const Vertex<T>& ver);
};

#endif
