#pragma once
#include <rog_map/prob_map.h>
#include <rog_map_msgs/LocalMap.h>

namespace rog_map {
// Single-writer mapping core. ROS callbacks and visualization never query it.
class LocalMapper : public ProbMap {
public:
    explicit LocalMapper(const ros::NodeHandle& nh) {
        cfg_ = Config(nh);
        if (!cfg_.map_sliding_en || cfg_.ros_callback_en || cfg_.batch_update_size != 1 ||
            cfg_.esdf_en || cfg_.frontier_extraction_en || cfg_.load_pcd_en || cfg_.unk_inflation_en ||
            cfg_.virtual_height_enable || !cfg_.inflation_cube ||
            std::abs(cfg_.resolution - cfg_.inflation_resolution) > 1e-9)
            throw std::invalid_argument("LocalMapper requires sliding, same-resolution cube inflation and no auxiliary/global maps");
        initProbMap();
    }

    void integrate(const PointCloud& cloud, const Vec3f& sensor) {
        if (!sensor.allFinite() || (sensor.array().abs() / cfg_.resolution > 100000000).any())
            throw std::invalid_argument("Invalid sensor position");
        updateProbMap(cloud, Pose(sensor, Quatf::Identity()));
    }

    void clear() {
        resetLocalMap();
        inf_map_->resetLocalMap();
    }

    void snapshot(rog_map_msgs::LocalMap& msg) const {
        msg.resolution = sc_.resolution;
        msg.origin.x = local_map_bound_min_i_.x() * sc_.resolution;
        msg.origin.y = local_map_bound_min_i_.y() * sc_.resolution;
        msg.origin.z = local_map_bound_min_i_.z() * sc_.resolution;
        for (int a = 0; a < 3; ++a) msg.size[a] = sc_.map_size_i[a];
        msg.inflation_steps = cfg_.inflation_step;
        exportBits(local_map_bound_min_i_, sc_.map_size_i, msg.occupied_bits,
                   [this](int hash) { return occupancy_buffer_[hash] >= cfg_.l_occ; });
        inf_map_->exportOccupiedBits(local_map_bound_min_i_, sc_.map_size_i, msg.inflated_bits);
    }
};
}
