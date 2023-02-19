//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_VIEWPOINT_H
#define ROBO_PLANNER_WS_VIEWPOINT_H

#include "point3d.h"

namespace utils {
    using Viewpoint = Point3D;

    using ViewpointSet = Point3DSet;
    template<typename T>
    using ViewpointMap = Point3DMap<T>;
    using ViewpointQueue = Point3DQueue;
}

#endif //ROBO_PLANNER_WS_VIEWPOINT_H

