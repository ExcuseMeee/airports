#include <string>
#include "Stack.h"

template<typename T>
Stack<T>::Stack() {}

template<typename T>
void Stack<T>::push(const T& val) {
  this->data.push_back(val);
}

template<typename T>
void Stack<T>::pop() {
  if(this->empty()) throw std::string("[pop] empty stack");
  this->data.pop_back();
}

template<typename T>
T& Stack<T>::top() {
  if(this->empty()) throw std::string("[top] empty stack");
  return this->data.back();
}

template<typename T>
bool Stack<T>::empty() const {
  return this->data.empty();
}