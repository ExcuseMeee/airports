#ifndef AIRPORTDATA_H
#define AIRPORTDATA_H
#include <string>
#include <iostream>
class AirportData {
public:
  AirportData(std::string airport, std::string city, std::string stateCode) {
    this->airport = airport;
    this->city = city;
    this->stateCode = stateCode;
  }

  bool operator==(const AirportData& other) const {
    return this->airport == other.airport;
  }

  friend std::ostream& operator<<(std::ostream& os, const AirportData& airportData) {
    os << airportData.airport;
    return os;
  }

  std::string airport;
  std::string stateCode;
  std::string city;
};


#endif