#pragma once

namespace asensing
{
namespace lidar
{


class DistanceSection
{
public:
  DistanceSection (float min, float max, float usr_min, float usr_max)
    : min_(min), max_(max)
  {
    if (usr_min < 0) usr_min = 0;
    if (usr_max < 0) usr_max = 0;

    if ((usr_min != 0) || (usr_max != 0))
    {
      min_ = usr_min;
      max_ = usr_max;
    }
  }

  bool in(float distance)
  {
    return ((min_ <= distance) && (distance <= max_));
  }

#ifndef UNIT_TEST
private:
#endif

  float min_;
  float max_;
};

}  // namespace lidar
}  // namespace asensing
