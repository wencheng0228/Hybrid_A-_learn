/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_HYBRID_A_STAR_H
#define HYBRID_A_STAR_HYBRID_A_STAR_H

#include <glog/logging.h>
#include <map>
#include <memory>
#include "rs_path.h"
#include "state_node.h"

class HybridAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HybridAStar() = delete;

    HybridAStar(double steering_angle,
                int steering_angle_discrete_num,
                double segment_length,
                int segment_length_discrete_num,
                double wheel_base,
                double steering_penalty,
                double reversing_penalty,
                double steering_change_penalty,
                double shot_distance,
                int grid_size_phi = 72);

    ~HybridAStar();

    void Init(double x_lower,
              double x_upper,
              double y_lower,
              double y_upper,
              double state_grid_resolution,
              double map_grid_resolution = 0.1);

    bool Search(const Vec3d& start_state, const Vec3d& goal_state);

    VectorVec4d GetSearchedTree();

    VectorVec3d GetPath() const;

    __attribute__((unused)) int GetVisitedNodesNumber() const { return visited_node_number_; }

    __attribute__((unused)) double GetPathLength() const;

    __attribute__((unused)) Vec2d CoordinateRounding(const Vec2d& pt) const;

    Vec2i Coordinate2MapGridIndex(const Vec2d& pt) const;

    void SetObstacle(double pt_x, double pt_y);

    void SetObstacle(unsigned int x, unsigned int y);

    /*!
     * Set vehicle shape
     * Consider the shape of the vehicle as a rectangle.
     * @param length vehicle length (a to c)
     * @param width vehicle width (a to d)
     * @param rear_axle_dist Length from rear axle to rear (a to b)
     *
     *         b
     *  a  ---------------- c
     *    |    |          |    Front
     *    |    |          |
     *  d  ----------------
     */
    void SetVehicleShape(double length, double width, double rear_axle_dist);

    void Reset();

private:
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;

    inline bool HasObstacle(const Vec2i& grid_index) const;

    bool CheckCollision(const double& x, const double& y, const double& theta);

    inline bool LineCheck(double x0, double y0, double x1, double y1);

    bool AnalyticExpansions(const StateNode::Ptr& current_node_ptr,
                            const StateNode::Ptr& goal_node_ptr,
                            double& length);

    inline double ComputeG(const StateNode::Ptr& current_node_ptr, const StateNode::Ptr& neighbor_node_ptr) const;

    inline double ComputeH(const StateNode::Ptr& current_node_ptr, const StateNode::Ptr& terminal_node_ptr);

    inline Vec3i State2Index(const Vec3d& state) const;

    inline Vec2d MapGridIndex2Coordinate(const Vec2i& grid_index) const;

    void GetNeighborNodes(const StateNode::Ptr& curr_node_ptr, std::vector<StateNode::Ptr>& neighbor_nodes);

    /*!
     * Simplified car model. Center of the rear axle
     * refer to: http://planning.cs.uiuc.edu/node658.html
     * @param step_size Length of discrete steps
     * @param phi Car steering angle
     * @param x Car position (world frame)
     * @param y Car position (world frame)
     * @param theta Car yaw (world frame)
     */
    // 计算固定距离内车辆的全局位姿变化
    inline void DynamicModel(const double& step_size, const double& phi, double& x, double& y, double& theta) const;

    static inline double Mod2Pi(const double& x);

    bool BeyondBoundary(const Vec2d& pt) const;

    void ReleaseMemory();

private:
    uint8_t* map_data_ = nullptr;
    double STATE_GRID_RESOLUTION_{};  // 状态网格的分辨率，用于关注在哪里搜索，分辨率通常较低
    double MAP_GRID_RESOLUTION_{};    // 地图网格的分辨率，用于关注哪里有障碍物，分辨率通常较高
    double ANGULAR_RESOLUTION_{};     // 角度分辨率
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr*** state_node_map_ = nullptr;

    std::multimap<double, StateNode::Ptr> openset_;

    double wheel_base_;                 // 轴距
    double segment_length_;             // 路径段长度，每次拓展节点时移动的距离
    double move_step_size_;             // 移动步长，= segment_length_ / segment_length_discrete_num_
    double steering_radian_step_size_;  // 单个档位的转向角大小 = steering_radian_ / steering_discrete_num_
    double steering_radian_;            // 最大转向角
    double tie_breaker_;                // 打破平局系数，用于避免相同代价的节点

    double shot_distance_;             // 启发式距离，当距离目标点小于3倍的该值时使用RS曲线计算更精确的启发值
    int segment_length_discrete_num_;  // 路径段的离散化数量，用于将 segment_length_ 离散成小段进行碰撞检测
    int steering_discrete_num_;        // 转向角离散化数量，或称为转向档位，也即把最大转向角分成多少份
    double steering_penalty_;          // 转向惩罚系数，使得算法更倾向于直线行驶而非频繁转向
    double reversing_penalty_;         // 倒车惩罚系数，使得算法更倾向于前进而非倒车
    double steering_change_penalty_;   // 转向改变的惩罚系数，惩罚角度变化以获得平滑路径

    double path_length_ = 0.0;  // 当前规划路径的总长度

    std::shared_ptr<RSPath> rs_path_ptr_;  // Reeds_shepp路径规划器的指针

    VecXd vehicle_shape_;           // 车辆形状的顶点坐标
    MatXd vehicle_shape_discrete_;  // 车辆形状的离散化点集，用于精确的碰撞检测

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif  // HYBRID_A_STAR_HYBRID_A_STAR_H
