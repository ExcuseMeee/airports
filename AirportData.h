#ifndef AIRPORTDATA_H
#define AIRPORTDATA_H
#include <string>

class AirportData {
public:
  AirportData(std::string airport, std::string stateCode, std::string city) {
    this->airport = airport;
    this->stateCode = stateCode;
    this->city = city;
  }

  bool operator==(const AirportData& other) const {
    return this->airport == other.airport;
  }

  std::string airport;
  std::string stateCode;
  std::string city;
};


#endif