#include <iostream>
#include "MinHeap.h"

template<typename T>
MinHeap<T>::MinHeap() {};

template<typename T>
void MinHeap<T>::swap(T& v1, T& v2) {
  T tmp = v1;
  v1 = v2;
  v2 = tmp;
}

template<typename T>
void MinHeap<T>::insert(const T& val) {
  this->data.push_back(val);

  int i = this->data.size() - 1; // index of last node (this one we just inserted)

  // percolate UP
  while (data[i] < data[(i - 1) / 2]) {
    swap(data[i], data[(i - 1) / 2]);
    i = (i - 1) / 2;
  }
}

template<typename T>
void MinHeap<T>::percolateDown(int index) {
  if (index < 0 || index >= this->data.size()) return;

  int curIndex = index;
  int newIndex = index;
  do {

    // determine which child to attempt swap with
    if (curIndex * 2 + 1 >= data.size()) {
      // if searching for left kid is out of bounds, then right kid definitely doesnt exist either. no kids to swap with.
      break;
    }
    else if (curIndex * 2 + 2 >= data.size()) {
      // no right kid, only left kid
      newIndex = curIndex * 2 + 1;
    }
    else if (curIndex * 2 + 2 < data.size()) {
      // both kids exist
      int leftVal = data[curIndex * 2 + 1];
      int rightVal = data[curIndex * 2 + 2];

      newIndex = leftVal < rightVal ? curIndex * 2 + 1 : curIndex * 2 + 2;
    }

    // perform swap IF child is less
    if (data[newIndex] < data[curIndex]) {
      swap(data[curIndex], data[newIndex]);
      curIndex = newIndex;
    }
    else {
      // if child is NOT less, then we cannot percolate any further
      break;
    }

  } while (true);
}

template<typename T>
T MinHeap<T>::popMin() {
  if (this->data.empty()) throw std::string("[popMin] empty heap");

  T return_val = data[0]; // min val should be at root for a minheap

  // replace root node with last node, then percolate down
  data[0] = data[data.size() - 1];
  data.pop_back(); // remove last node (we are moved it to index 0)

  percolateDown(0); // perc down from root

  return return_val;
}