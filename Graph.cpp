#include "Graph.h"
#include "Vertex.h"
#include <queue>
#include <unistd.h> //check if this is allowed
#include <climits> //check if this is allowed
#include <iostream>
#include "MinHeap.h"
#include "MinHeap.cpp"

template <typename T>
void Graph<T>::insert_vertex(const Vertex<T>& ver) {
    if (get_vertex_index(ver) == -1) {
        vertices.push_back(ver); //insert the vertex to the array of vertices
        std::vector<Edge> tmp;
        edges.push_back(tmp); //insert empty vector to the edges
    }
}

template <typename T>
int Graph<T>::get_vertex_index(const Vertex<T>& ver) {
    for(int i = 0; i < vertices.size(); i++) {
        if (vertices[i].getData() == ver.getData()) {
            return i;
        }
    }

    return -1;
}

template <typename T>
void Graph<T>::add_edge(const Vertex<T>& ver1, const Vertex<T>& ver2, int weight) {
    int i1 = get_vertex_index(ver1);
    int i2 = get_vertex_index(ver2);
    if (i1 == -1 || i2 == -1) {
        throw std::string("Add_edge: incorrect vertices");
    }
    Edge v(i1, i2, weight);
    Edge v2(i2, i1, weight);
    edges[i1].push_back(v);
    if (i1 != i2) {
        edges[i2].push_back(v2);
    }

}

template <typename T>
void Graph<T>::print() const {
    for (int i = 0; i < vertices.size(); i++) {
        std::cout << "{ " << vertices[i].getData() << ": ";
        for(int j = 0; j < edges[i].size(); j++) {
            std::cout << '{' << vertices[edges[i][j].dest].getData() << ", ";
            std::cout << edges[i][j].weight << "} ";
        }
        std::cout << " }\n";
    }
}

template <typename T>
void Graph<T>::DFS(Vertex<T>& ver) {
    clean_visited();
    DFS_helper(ver);
    clean_visited();
}

template <typename T>
void print_queue(std::queue<Vertex<T>> q) {
  while (!q.empty())
  {
    std::cout << q.front().getData() << " ";
    q.pop();
  }
  std::cout << std::endl;
}

template <typename T>
void Graph<T>::BFS(Vertex<T>& ver) {
    clean_visited();

    int i = get_vertex_index(ver);
    if (i == -1) {
        throw std::string("BFS: Vertex is not in the graph");
    }
    std::queue<int> q;
    q.push(i);
    vertices[i].setVisited(true);

    while (!q.empty()) {
        int i = q.front();
        std::cout << vertices[i].getData() << ' ';
        for(int j = 0; j < edges[i].size(); j++) {
            int adjacent_ver = edges[i][j].dest;
            if (vertices[adjacent_ver].getVisited() == false) {
                vertices[adjacent_ver].setVisited(true);
                q.push(adjacent_ver);
            }
        }
        q.pop();
    }

    clean_visited();
}

template <typename T>
void Graph<T>::clean_visited() {
    for(Vertex<T>& v : vertices) {
        v.setVisited(false);
    }
}


template <typename T>
void Graph<T>::DFS_helper(Vertex<T>& ver) {
    int i = get_vertex_index(ver);
    if (i == -1) {
        throw std::string("DFS: Vertex is not in the graph");
    }
    vertices[i].setVisited(true); //visit the current vertex
    std::cout << vertices[i].getData() << ' '; //print the data of the current vertex
    for(int j = 0; j < edges[i].size(); j++) {
        Vertex<T> adjacent_ver = vertices[edges[i][j].dest];
        if (adjacent_ver.getVisited() == false) {
            DFS_helper(adjacent_ver);
        }
    }
}

template <typename T>
int Graph<T>::shortest_path(const Vertex<T>& src, const Vertex<T>& dest) {
  int i_src = get_vertex_index(src);
  int i_dest = get_vertex_index(dest);

  if (i_src == -1 || i_dest == -1) {
    throw std::string("Shortest path: incorrect vertices");
  }

  clean_visited();
  std::vector<int> distances(vertices.size()); //represents the distances from source to all other vertices

  //set initial distances
  for(int i = 0; i < distances.size(); i++) {
    distances[i] = (i == i_src) ? 0 : INT_MAX;
  }
  std::queue<int> q;
  q.push(i_src);

  while (!q.empty()) {
    int i = q.front();
    for(int j = 0; j < edges[i].size(); j++) {
      int i_adjacent_ver = edges[i][j].dest;
      
      if (vertices[i_adjacent_ver].getVisited() == false) {
        q.push(i_adjacent_ver);
        int dist_to_neighbour = distances[i] + edges[i][j].weight;
        
        if (dist_to_neighbour < distances[i_adjacent_ver]) {
          distances[i_adjacent_ver] = dist_to_neighbour;
          for(int i : distances) {
            std::cout << i << " ";
          }
          std::cout << "\n";
        }
        
      }
      
    }
    vertices[i].setVisited(true);
    q.pop();
  }

  clean_visited();

  return distances[i_dest];
}


template <typename T>
int Graph<T>::dijkstra_shortest_path(const Vertex<T>& src, const Vertex<T>& dest) {
  int i_src = get_vertex_index(src);
  int i_dest = get_vertex_index(dest);

  if (i_src == -1 || i_dest == -1) {
    throw std::string("Shortest path: incorrect vertices");
  }

  clean_visited();
  std::vector<int> distances(vertices.size()); //represents the distances from source to all other vertices

  //set initial distances
  for(int i = 0; i < distances.size(); i++) {
    distances[i] = (i == i_src) ? 0 : INT_MAX;
  }

  //minheap
  MinHeap<Edge> heap;
  int vertices_visited = 0;
  int cur_ver = i_src;

  while (vertices_visited < vertices.size()) {
    int i = cur_ver;
    //check the neighbors of the cur vertex
    for(int j = 0; j < edges[i].size(); j++) {
      int i_adjacent_ver = edges[i][j].dest;

      if (vertices[i_adjacent_ver].getVisited() == false) {
        heap.insert(edges[i][j]); //insert the edge to the heap of edges
        int dist_to_neighbour = distances[i] + edges[i][j].weight;

        if (dist_to_neighbour < distances[i_adjacent_ver]) {
          distances[i_adjacent_ver] = dist_to_neighbour;
          for(int i : distances) {
            std::cout << i << " ";
          }
          std::cout << "\n";
        }

      }

    }
    Edge e = heap.delete_min();
    cur_ver = e.dest;
    vertices[i].setVisited(true);
    vertices_visited++;
  }

  clean_visited();

  return distances[i_dest];
}
