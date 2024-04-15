#ifndef VERTEX_H
#define VERTEX_H

template<typename T>
class Vertex {
public:
  Vertex(const T& data = T()) {
    this->data = data;
  }
  const T& getData() const {
    return this->data;
  }
  const bool getVisited() const {
    return this->visited;
  }
  void setVisited(const bool v) {
    this->visited = v;
  }

private:
  T data;
  bool visited;
};

#endif
