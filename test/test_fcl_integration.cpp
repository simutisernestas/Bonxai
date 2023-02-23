#include <iostream>
#include "bonxai/bonxai.hpp"
#include "fcl/geometry/collision_geometry.h"
#include "fcl/narrowphase/collision_object.h"
#include <bitset>

namespace Bonxai
{

template <typename S>
class BonxaiCollisionGeometry : public fcl::CollisionGeometry<S>
{
private:
  std::shared_ptr<Bonxai::VoxelGrid<S>> grid_;

public:
  BonxaiCollisionGeometry(const S resolution);
  BonxaiCollisionGeometry(const std::shared_ptr<Bonxai::VoxelGrid<S>> grid_ptr);

  void computeLocalAABB();
};

template <typename S>
BonxaiCollisionGeometry<S>::BonxaiCollisionGeometry(S resolution)
  : grid_(std::make_shared<Bonxai::VoxelGrid<S>>(resolution))
{
}

template <typename S>
BonxaiCollisionGeometry<S>::BonxaiCollisionGeometry(
    const std::shared_ptr<Bonxai::VoxelGrid<S>> grid_ptr)
{
  grid_ = grid_ptr;
}

template <typename S>
void BonxaiCollisionGeometry<S>::computeLocalAABB()
{
  grid_;
  fcl::AABB<S> aabb;
  this->aabb_local = aabb;
  this->aabb_center = this->aabb_local.center();
  this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
  std::cout << "computeLocalAABB" << std::endl;
}

}  // namespace Bonxai

int main(int argc, char** argv)
{
  const double VOXEL_RESOLUTION = .1;

  Bonxai::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto accessor = grid.createAccessor();

  auto value_ptr = accessor.value({});
  if (value_ptr == nullptr)
    std::cout << "Empty as expected" << std::endl;

  int count = 0;
  for (double x = -0.5; x < 0.5; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.5; y < 0.5; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.5; z < 0.5; z += VOXEL_RESOLUTION)
      {
        accessor.setValue(grid.posToCoord(x, y, z), count++);
      }
    }
  }

  bool fail = false;
  for (double x = -0.5; x < 0.5; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.5; y < 0.5; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.5; z < 0.5; z += VOXEL_RESOLUTION)
      {
        int* value = accessor.value(grid.posToCoord(x, y, z));
        if (value == nullptr)
        {
          std::cout << "Problem at coordinate " << x << " " << y << " " << z
                    << std::endl;
          fail = true;
        }
      }
    }
  }

  if (fail)
    return 1;

  auto cg =
      std::make_shared<Bonxai::BonxaiCollisionGeometry<double>>(VOXEL_RESOLUTION);
  fcl::CollisionObject<double> tree_obj(cg);

  std::cout << (1 << 3) << std::endl;

  uint8_t LEAF_BITS = 3;
  auto intr = (1 << LEAF_BITS);
  auto intr2 = (intr - 1);
  const int32_t MASK = ~intr2;
  std::cout << std::bitset<32>(intr) << std::endl;
  std::cout << std::bitset<32>(intr2) << std::endl;
  std::cout << std::bitset<32>(MASK) << std::endl;

  std::cout << grid.memUsage() << std::endl;

  // TODO: could have a simple brute force method to check the boundaries for AABB
  // readme has references on implementation : ) https://github.com/facontidavide/Bonxai
  // https://www.geeksforgeeks.org/octree-insertion-and-searching/ 
  // https://castle-engine.io/vrml_engine_doc/output/xsl/html/section.how_octree_works.html
  
  return 0;
}