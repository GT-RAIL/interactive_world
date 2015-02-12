#ifndef SPATIAL_WORLD_MODEL_POINT_OF_INTEREST_H_
#define SPATIAL_WORLD_MODEL_POINT_OF_INTEREST_H_

#include <string>

namespace spatial_world_model
{

class PointOfInterest
{
public:
  PointOfInterest(std::string name, std::string frame)
  {
    name_ = name;
    frame_ = frame;
  };

  std::string getName() const
  {
    return name_;
  }

  void setName(std::string name)
  {
    name_ = name;
  }

  std::string getFrame() const
  {
    return frame_;
  }

  void setFrame(std::string frame)
  {
    frame_ = frame;
  }

private:
  std::string name_, frame_;
};

}

#endif
