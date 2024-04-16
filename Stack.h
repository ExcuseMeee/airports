#ifndef STACK_H
#define STACK_H
#include <vector>

template<typename T>
class Stack {
public:
  Stack();

  void push(const T& val);
  void pop();
  T& top();

  bool empty() const;


private:
  std::vector<T> data;
};

#endif