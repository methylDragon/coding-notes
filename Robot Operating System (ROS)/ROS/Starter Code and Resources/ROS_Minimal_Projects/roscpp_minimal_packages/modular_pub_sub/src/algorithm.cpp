#include "PACKAGE_HEADERS/algorithm.hpp"

namespace node_class_interface {

// Here's the implementation
// The private variables are listed in the header file (.hpp)
Algorithm::Algorithm()
    : average_(0.0),
      nMeasurements_(0)
{
}

Algorithm::~Algorithm()
{
}

// When data is added, recalculate the average
void Algorithm::addData(const double data)
{
  average_ = (nMeasurements_ * average_ + data) / (nMeasurements_ + 1);
  nMeasurements_++;
}

// Getter method
double Algorithm::getAverage() const
{
  return average_;
}

} /* namespace */
