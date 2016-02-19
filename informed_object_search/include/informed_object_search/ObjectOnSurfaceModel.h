#ifndef INTERACTIVE_WORLD_OBJECT_ON_SURFACE_H_
#define INTERACTIVE_WORLD_OBJECT_ON_SURFACE_H_

#include <string>
#include <vector>

namespace rail
{
namespace interactive_world
{

struct observation
{
  time_t seen;
  time_t removed;
};

class ObjectOnSurfaceModel
{
public:
  ObjectOnSurfaceModel(const std::string &object = "", const std::string &surface = "");

  const std::string &getObject() const;

  void setObject(const std::string &object);

  const std::string &getSurface() const;

  void setSurface(const std::string &surface);

  time_t getLastSeen() const;

  void addObservation(const time_t seen, const time_t removed);

  double getMu();

  double getLambda();

  double getCurrentProbability();

private:
  std::string object_, surface_;
  time_t last_seen_;
  std::vector<struct observation> observations_;
  bool dirty_;
  double mu_;
};

}
}

#endif
