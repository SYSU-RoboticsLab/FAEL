//
// Created by hjl on 2022/6/7.
//

#ifndef ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H
#define ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H

#include <cmath>
#include <vector>

class Two_Opt {
public:
    std::vector<double> points_gains_;
    std::vector<std::vector<double>> cost_matrix_;
    double lambda_;
    std::vector<int> init_route_;

    std::vector<int> best_route_;
    double best_unity_;
    int searched_route_num_;

    Two_Opt(std::vector<int> &init_route, const std::vector<double> &point_gains,
            const std::vector<std::vector<double>> &cost_matrix, const double &lambda) {
        init_route_ = init_route;
        points_gains_ = point_gains;
        cost_matrix_ = cost_matrix;
        lambda_ = lambda;

        searched_route_num_=0;
    }

    void solve() {
        auto current_route = init_route_;
        auto best_route = current_route;
        auto best_unity = computeUnity(best_route, points_gains_, cost_matrix_, lambda_);
        auto new_route = best_route;
        auto new_unity = best_unity;
        bool is_improved = true;
        while (is_improved) {
            is_improved = false;
            for (int i = 1; i < current_route.size() - 1; ++i) {
                for (int j = i + 1; j < current_route.size(); ++j) {
                    new_route = twoOptSwap(current_route, i, j);
                    new_unity = computeUnity(new_route, points_gains_, cost_matrix_, lambda_);
                    searched_route_num_++;
                    if (new_unity > best_unity) {
                        best_route = new_route;
                        best_unity = new_unity;
                        is_improved = true;
                    }
                }
            }
            current_route = best_route;
        }
        best_route_ = current_route;
        best_unity_ = best_unity;
    }

    double computeUnity(const std::vector<int> &route, const std::vector<double> &point_gains,
                        const std::vector<std::vector<double>> &cost_matrix, const double &lambda) {
        double current_cost = 0.0;
        double current_unity = 0.0;
        for (int i = 1; i < route.size(); i++) {
            current_cost = current_cost + cost_matrix[route[i - 1]][route[i]];
            current_unity = current_unity + point_gains[route[i]] * exp(-lambda * current_cost);
        }
        return current_unity;
    }

    std::vector<int> twoOptSwap(const std::vector<int> &route, const int &i, const int &j) {
        std::vector<int> swap_route = route;
        std::reverse(swap_route.begin()+i,swap_route.begin()+j+1);

        return swap_route;
    }

};


#endif //ROBO_PLANNER_WS_TEST_NEW_TWO_OPT_H
