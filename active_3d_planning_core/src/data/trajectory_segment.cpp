#include <memory>
#include <vector>

#include <active_3d_planning_core/data/trajectory_segment.h>

namespace active_3d_planning {

TrajectorySegment::TrajectorySegment()
    : parent(nullptr), tg_visited(false), gain(0.0), cost(0.0), value(0.0), in_range(0) {}

bool TrajectorySegment::compare(TrajectorySegment a, TrajectorySegment b) {
  return (a.value < b.value);
}

bool TrajectorySegment::comparePtr(TrajectorySegment* a, TrajectorySegment* b) {
  return (a->value < b->value);
}

TrajectorySegment* TrajectorySegment::spawnChild() {
  children.push_back(
      std::unique_ptr<TrajectorySegment>(new TrajectorySegment()));
  children.back()->parent = this;
  return children.back().get();
}

void TrajectorySegment::getChildren(std::vector<TrajectorySegment*>* result) {
  for (int i = 0; i < children.size(); ++i) {
    result->push_back(children[i].get());
  }
}

void TrajectorySegment::getLeaves(std::vector<TrajectorySegment*>* result) {
  if (children.empty()) {
    result->push_back(this);
    return;
  }
  for (int i = 0; i < children.size(); ++i) {
    children[i]->getLeaves(result);
  }
}

void TrajectorySegment::getTree(std::vector<TrajectorySegment*>* result) {
  result->push_back(this);
  for (int i = 0; i < children.size(); ++i) {
    children[i]->getTree(result);
  }
}

void TrajectorySegment::getTree(std::vector<TrajectorySegment*>* result,
                                int maxdepth) {
  result->push_back(this);
  if (maxdepth > 0) {
    for (int i = 0; i < children.size(); ++i) {
      children[i]->getTree(result, maxdepth - 1);
    }
  }
}

TrajectorySegment TrajectorySegment::shallowCopy() {
  TrajectorySegment copy = TrajectorySegment();
  copy.trajectory = trajectory;
  copy.gain = gain;
  copy.cost = cost;
  copy.value = value;
  copy.tg_visited = tg_visited;
  copy.parent = parent;
  return copy;
}

int TrajectorySegment::checkSegmentsAreWithinRange(Eigen::Vector3d target, double radius) {
  // check if children (if any) are within range
  in_range = 0;
  for (int i = 0; i < children.size(); ++i) {
    in_range += children[i]->checkSegmentsAreWithinRange(target, radius);
  }

  // check if this node is within range
  Eigen::Vector3d node = trajectory.back().position_W;
  Eigen::Vector2d target_xy = target.segment(0, 2);
  Eigen::Vector2d node_xy = node.segment(0, 2);
  Eigen::Vector2d diff_xy = target_xy - node_xy;
  if (diff_xy.norm() < radius) {
    //trajectory.back().position_W = target;
    in_range++;
  }
  return in_range;
}

}  // namespace active_3d_planning
