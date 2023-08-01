#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/subsequent_best.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBest
ModuleFactoryRegistry::Registration<SubsequentBest>
    SubsequentBest::registration("SubsequentBest");

SubsequentBest::SubsequentBest(PlannerI& planner) : NextSelector(planner) {}

void SubsequentBest::setupFromParamMap(Module::ParamMap* param_map) {}

int SubsequentBest::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates = {0};
  if (traj_in->in_range > 0) {
    std::cout << "There are " << traj_in->in_range << " nodes in range of waypoint - Only evaluating nodes within range" << std::endl;
    double current_max = -1;
    for (int i = 0; i < traj_in->children.size(); ++i) {
      if (traj_in->children[i]->in_range) {
        double current_value = evaluateSingle(traj_in->children[i].get());
        if (current_value > current_max) {
          current_max = current_value;
          candidates.clear();
          candidates.push_back(i);
        } else if (current_value == current_max) {
          candidates.push_back(i);
        }
      }
    }
    if (current_max < 0)
      std::cout << "this should not be possible!!!" << std::endl;
  }
  else {
    double current_max = evaluateSingle(traj_in->children[0].get());
    for (int i = 1; i < traj_in->children.size(); ++i) {
      double current_value = evaluateSingle(traj_in->children[i].get());
      if (current_value > current_max) {
        current_max = current_value;
        candidates.clear();
        candidates.push_back(i);
      } else if (current_value == current_max) {
        candidates.push_back(i);
      }
    }
  }

  // randomize if multiple maxima
  std::random_shuffle(candidates.begin(), candidates.end());
  return candidates[0];
}

TrajectorySegment* SubsequentBest::selectNextBestWholeTraj(const TrajectorySegment* traj_in,
                                            Eigen::Vector3d target, double radius) {
  TrajectorySegment* traj_out;
  // add to a vector all traj segments with the back close to the current waypoint
  std::vector<TrajectorySegment*> candidates_seg;
  gatherInRangeSeg(&candidates_seg, traj_in, target, radius);
  if (candidates_seg.size() > 0) {
    // and check the value
    // assign the traj segments with the highest value to traj_out -> keep moving until reaching the back of this
    std::vector<int> candidates = {0};
    double current_max = evaluateSingle(candidates_seg[0]);
    for (int i = 1; i < candidates_seg.size(); ++i) {
      double current_value = evaluateSingle(candidates_seg[i]);
      if (current_value > current_max) {
        current_max = current_value;
        candidates.clear();
        candidates.push_back(i);
      } else if (current_value == current_max) {
        candidates.push_back(i);
      }
    }
    traj_out = candidates_seg[candidates[0]];
  }
  else {
    // assign the IMMEDIATE traj segments with the highest value to traj_out (original code)
    std::vector<int> candidates = {0};
    double current_max = evaluateSingle(traj_in->children[0].get());
    for (int i = 1; i < traj_in->children.size(); ++i) {
      double current_value = evaluateSingle(traj_in->children[i].get());
      if (current_value > current_max) {
        current_max = current_value;
        candidates.clear();
        candidates.push_back(i);
      } else if (current_value == current_max) {
        candidates.push_back(i);
      }
    }
    // randomize if multiple maxima
    std::random_shuffle(candidates.begin(), candidates.end());
    traj_out = traj_in->children[candidates[0]].get();
  }
  return traj_out;
}

void SubsequentBest::gatherInRangeSeg(std::vector<TrajectorySegment*>* candidates_seg, const TrajectorySegment* traj_in, 
                                      Eigen::Vector3d target, double radius) {
  for (int i = 0; i < traj_in->children.size(); ++i) {
    if (traj_in->children[i].get()->checkSegmentsAreWithinRange(target, radius, false) > 0) {
      candidates_seg->push_back(traj_in->children[i].get());
    }
    gatherInRangeSeg(candidates_seg, (const TrajectorySegment*)traj_in->children[i].get(), target, radius);
  }  
}

double SubsequentBest::evaluateSingle(TrajectorySegment* traj_in) {
  // Recursively find highest value
  if (traj_in->children.empty()) {
    return traj_in->value;
  }
  double highest_value = traj_in->value;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    highest_value =
        std::max(highest_value, evaluateSingle(traj_in->children[i].get()));
  }
  return highest_value;
}

}  // namespace next_selector
}  // namespace active_3d_planning
