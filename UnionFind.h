#ifndef UNIONFIND_H
#define UNIONFIND_H
#include <iostream>
#include <vector>

class UnionFind {
public:
    UnionFind(int size);

    int find(int n);

    void unionFunction(int n, int n2);
private:
std::vector<int> parent;
std::vector<int> rank;
};

#endif
