#include "UnionFind.h"
#include <iostream>
#include <vector>


UnionFind::UnionFind(int size) {
  parent.resize(size);
  rank.resize(size, 0);
  for (int i = 0; i < size; i++) {
      parent[i] = i;
  }
}

int UnionFind::find(int n) {
    if (parent[n] == n) { //if input is it's own parent
        return n;
    }
    else { //recursively find parent
        return find(parent[n]);
    }
}

void UnionFind::unionFunction(int n, int n2) {
  
  int nRoot = find(n); //finds root node of n

  int n2Root = find(n2); //finds root node of n2

  parent[nRoot] = n2Root;
}
