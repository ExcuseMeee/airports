#ifndef MINHEAP_H
#define MINHEAP_H 
#include <vector>

template<typename T>
class MinHeap {
public:
  MinHeap();
  void insert(const T& val);
  T popMin();

  bool empty() const;

private:
  std::vector<T> data;

  void swap(T& v1, T& v2);
  void percolateDown(int index);

};


#endif
