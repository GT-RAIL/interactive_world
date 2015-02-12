#ifndef SPATIAL_WORLD_MODEL_SURFACE_H_
#define SPATIAL_WORLD_MODEL_SURFACE_H_

#include <string>
#include <vector>
#include <spatial_world_model/PointOfInterest.h>

namespace spatial_world_model
{

class Surface
{
public:
  Surface(std::string name, std::string frame, double width, double height)
  {
    name_ = name;
    frame_ = frame;
    width_ = width;
    height_ = height;
  };

  std::string getName() const
  {
    return name_;
  }

  void setName(std::string name)
  {
    name_ = name;
  }

  double getWidth() const
  {
    return width_;
  }

  void setWidth(double width)
  {
    width_ = width;
  }

  double getHeight() const
  {
    return height_;
  }

  void setHeight(double height)
  {
    height_ = height;
  }

  std::string getFrame() const
  {
    return frame_;
  }

  void setFrame(std::string frame)
  {
    frame_ = frame;
  }

  size_t getNumPlacementSurfaces() const
  {
    return placement_surfaces_.size();
  }

  void addPlacementSurface(std::string frame)
  {
    placement_surfaces_.push_back(frame);
  }

  std::string &getPlacementSurface(size_t i)
  {
    return placement_surfaces_[i];
  }

  size_t getNumPOIs() const
  {
    return placement_surfaces_.size();
  }

  void addPOI(PointOfInterest poi)
  {
    pois_.push_back(poi);
  }

  PointOfInterest &getPOI(size_t i)
  {
    return pois_[i];
  }

private:
  std::string name_, frame_;
  std::vector<std::string> placement_surfaces_;
  std::vector<PointOfInterest> pois_;
  double width_, height_;
};

}

#endif
