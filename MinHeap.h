#ifndef MINHEAP_H
#define MINHEAP_H
#include <vector>


template <typename T>
class MinHeap {
public:
  MinHeap() {}
  void insert(const T& val);

  void print() const;
  T delete_min();


private:
  std::vector<T> data;

  void swap(T& v1, T& v2) {
    T tmp = v1;
    v1 = v2;
    v2 = tmp;
  }

  void percolate_down(int i);
  int min_index(int i1, int i2) const;
};


#endif
