//
// Created by hjl on 2022/1/9.
//

#ifndef ROBO_PLANNER_WS_MAP_H
#define ROBO_PLANNER_WS_MAP_H


#include <boost/shared_ptr.hpp>

#include "utils/frontier.h"

namespace perception {

    using namespace utils;

    class Map3D {
    public:
        typedef std::shared_ptr<Map3D> Ptr;

        Map3D()= default;

        virtual FrontierSet getFrontiers() const = 0;

        virtual bool isFrontier(const Frontier &frontier) const = 0;

    };

}

#endif //ROBO_PLANNER_WS_MAP_H


