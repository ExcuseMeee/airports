#include "MinHeap.h"
#include <iostream>


template<typename T>
void MinHeap<T>::insert(const T& val) {
    data.push_back(val);
    int n = data.size() - 1; //index of the last node
    //percolate UP
    while (data[n] < data[(n - 1) / 2]) {
        swap(data[n], data[(n - 1) / 2]);
        n = (n - 1) / 2; //make cur index equal to the parent index
    }
}

template<typename T>
void MinHeap<T>::print() const {
  int cur_level = 0;
  int new_level = 1;
    for(const T& i : data) {
      std::cout << i << ' ';
      cur_level++;
      if (cur_level == new_level) {
        std::cout <<"\n---------------------------\n";
        new_level *= 2;
        cur_level = 0;
      }
    }
    std::cout << '\n';
}


template<typename T>
T MinHeap<T>::delete_min() {
    if (data.empty()) {
        throw std::string("delete_min: Empty Heap\n");
    }    
    T res = data[0];
    data[0] = data[data.size() - 1]; //set the root with the value of the last node
    data.pop_back(); //deletes the last node

    //percolate DOWN
    percolate_down(0);
  return res;
}


template<typename T>
void MinHeap<T>::percolate_down(int i) {
    if (i >= data.size() || i < 0) {
        return;
    }

    int parent_index = i;
    int kids_min_index = i;

    do {
        //no kids
        if (parent_index * 2 + 1 >= data.size()) {
            break;
        }
        else if (parent_index * 2 + 2 < data.size()){ //has 2 kids
            kids_min_index = min_index(parent_index * 2 + 1, parent_index * 2 + 2);
        }
        else if (parent_index * 2 + 1 < data.size()){ //left kid
            kids_min_index = parent_index * 2 + 1;
        }

        //check is the smallest child smaller than the parent
        if (data[kids_min_index] < data[parent_index]) {
          swap(data[parent_index], data[kids_min_index]);
          parent_index = kids_min_index;
        }
        else {
          break;
        }
    } while(1);

}

template<typename T>
int MinHeap<T>::min_index(int i1, int i2) const {
  if (i1 >= data.size() || i2 >= data.size() || i1 < 0 || i2 < 0) {
    throw std::string ("min_index: incorrect index");
  }

  return (data[i1] < data[i2] ? i1 : i2);
}
