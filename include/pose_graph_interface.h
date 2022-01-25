#pragma once

#include <memory>

#include "submap/submap.h"
#include "transform.h"
#include "graph_node.h"

namespace mapping{

    class PoseGraphInterface{
    public:
           struct Constraint{
              struct Pose{
                 transform::Rigid3d zbar_ij;
                 double translation_weight;
                 double rotation_weight;
            };
           SubmapId submap_id;
           NodeId node_id;

           Pose pose;

           enum Tag
          {
            INTRA_SUBMAP,
            INTER_SUBMAP
          } tag;
     };

    enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

    struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
    };

    transform::Rigid3d relative_pose;

    };
}