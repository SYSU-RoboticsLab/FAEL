//
// Created by hjl on 2022/1/9.
//

#ifndef ROBO_PLANNER_WS_FRONTIER_H
#define ROBO_PLANNER_WS_FRONTIER_H

#include "point3d.h"

namespace utils {
    using Frontier = Point3D;

    using FrontierSet = Point3DSet;    //a collection representing frontier, can be well indexed and stored
    template<typename T>
    using FrontierMap = Point3DMap<T>;
    using FrontierQueue = Point3DQueue;
}


#endif //ROBO_PLANNER_WS_FRONTIER_H