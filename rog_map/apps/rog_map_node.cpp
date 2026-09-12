#include <rog_map/local_mapper.h>
#include <rog_map_msgs/validation.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/serialization.h>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Organized clouds and XYZ-only clouds are accepted without PCL missing-field
// warnings. Reject invalid layout before reading any data.
rog_map::PointCloud worldCloud(const sensor_msgs::PointCloud2& msg, const Eigen::Isometry3d& tf) {
    if (msg.is_bigendian || !msg.point_step ||
        uint64_t(msg.row_step) < uint64_t(msg.width) * msg.point_step ||
        uint64_t(msg.row_step) * msg.height > msg.data.size())
        throw std::invalid_argument("Invalid point cloud layout or unsupported big-endian cloud");
    sensor_msgs::PointField fields[3];
    const char* names[] = {"x", "y", "z"};
    for (int a = 0; a < 3; ++a) {
        bool found = false;
        for (const auto& field : msg.fields) if (field.name == names[a]) {
            if (found || field.count != 1 ||
                (field.datatype != sensor_msgs::PointField::FLOAT32 && field.datatype != sensor_msgs::PointField::FLOAT64))
                throw std::invalid_argument("Invalid XYZ field");
            const size_t width = field.datatype == sensor_msgs::PointField::FLOAT32 ? 4 : 8;
            if (uint64_t(field.offset) + width > msg.point_step) throw std::invalid_argument("XYZ outside point stride");
            fields[a] = field;
            found = true;
        }
        if (!found) throw std::invalid_argument("Missing XYZ field");
    }
    rog_map::PointCloud cloud;
    cloud.reserve(size_t(msg.width) * msg.height);
    for (uint32_t y = 0; y < msg.height; ++y) for (uint32_t x = 0; x < msg.width; ++x) {
        const uint8_t* point = msg.data.data() + size_t(y)*msg.row_step + size_t(x)*msg.point_step;
        Eigen::Vector3d p;
        for (int a = 0; a < 3; ++a) {
            if (fields[a].datatype == sensor_msgs::PointField::FLOAT32) {
                float v; std::memcpy(&v, point + fields[a].offset, 4); p[a] = v;
            } else std::memcpy(&p[a], point + fields[a].offset, 8);
        }
        if (!p.allFinite()) continue;
        p = tf * p;
        if (!p.allFinite()) continue;
        rog_map::PclPoint output{};
        output.x = p.x(); output.y = p.y(); output.z = p.z();
        if (std::isfinite(output.x) && std::isfinite(output.y) && std::isfinite(output.z)) cloud.push_back(output);
    }
    return cloud;
}

class LocalMapNode {
public:
    static ros::NodeHandle configure(ros::NodeHandle nh) {
        double radius, resolution;
        nh.param("dilate_radius", radius, 0.2);
        nh.param("rog_map/resolution", resolution, 0.1);
        if (!std::isfinite(radius) || radius < 0 || !std::isfinite(resolution) || resolution <= 0)
            throw std::invalid_argument("Invalid dilation radius or resolution");
        nh.setParam("rog_map/inflation_step", int(std::ceil(radius / resolution)));
        return nh;
    }
    LocalMapNode() : nh_("~"), listener_(tf_), mapper_(configure(nh_)) {
        nh_.param<std::string>("world_frame_id", world_, "world");
        nh_.param<std::string>("sensor_origin_frame_id", sensor_, "dragon/lidar_origin");
        nh_.param("tf_timeout", tf_timeout_, 0.05);
        if (world_.empty() || sensor_.empty() || !std::isfinite(tf_timeout_) || tf_timeout_ < 0)
            throw std::invalid_argument("Invalid local map TF configuration");
        newEpoch();
        map_pub_ = nh_.advertise<rog_map_msgs::LocalMap>("local_map", 1, false);
        viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1, false);
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
        cloud_sub_ = nh_.subscribe("cloud", 1, &LocalMapNode::cloudCallback, this, ros::TransportHints().tcpNoDelay());
        reset_srv_ = nh_.advertiseService("reset", &LocalMapNode::reset, this);
        viz_timer_ = nh_.createTimer(ros::Duration(0.5), &LocalMapNode::visualize, this);
        diag_timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &LocalMapNode::diagnose, this);
        worker_ = std::thread(&LocalMapNode::work, this);
    }
    ~LocalMapNode() {
        { std::lock_guard<std::mutex> lock(mutex_); stop_ = true; }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
private:
    void newEpoch() {
        epoch_ = std::max(epoch_ + 1, ros::WallTime::now().toNSec());
        version_ = 0;
    }
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        ++received_;
        if (cloud->header.stamp.isZero() || cloud->header.frame_id.empty()) { ++rejected_; return; }
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_) ++dropped_;
        pending_ = cloud;
        received_at_ = Clock::now();
        cv_.notify_one();
    }
    bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        std::unique_lock<std::mutex> lock(mutex_);
        pending_.reset();
        reset_requested_ = true;
        cv_.notify_one();
        reset_cv_.wait(lock, [this] { return !reset_requested_ || stop_; });
        return !stop_;
    }
    void publish(const ros::Time& stamp) {
        const auto start = Clock::now();
        auto map = boost::make_shared<rog_map_msgs::LocalMap>();
        mapper_.snapshot(*map);
        map->header.frame_id = world_;
        map->header.stamp = stamp;
        map->epoch = epoch_;
        map->version = ++version_;
        export_ms_ = ms(start);
        bytes_ = ros::serialization::serializationLength(*map);
        map_pub_.publish(map);
        { std::lock_guard<std::mutex> lock(snapshot_mutex_); snapshot_ = map; }
        ++published_;
    }
    void work() {
        while (true) {
            sensor_msgs::PointCloud2::ConstPtr cloud;
            Clock::time_point received;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || reset_requested_ || pending_; });
                if (stop_) return;
                if (reset_requested_) {
                    lock.unlock();
                    mapper_.clear(); newEpoch();
                    if (located_) publish(last_stamp_);
                    lock.lock(); reset_requested_ = false; reset_cv_.notify_all();
                    continue;
                }
                cloud.swap(pending_);
                received = received_at_;
            }
            try {
                const auto tf_start = Clock::now();
                const auto transform = tf_.lookupTransform(world_, cloud->header.frame_id, cloud->header.stamp,
                                                         ros::Duration(tf_timeout_));
                const auto origin = tf_.lookupTransform(world_, sensor_, cloud->header.stamp, ros::Duration(tf_timeout_));
                tf_ms_ = ms(tf_start);
                const auto decode_start = Clock::now();
                auto points = worldCloud(*cloud, tf2::transformToEigen(transform));
                transform_ms_ = ms(decode_start);
                const auto& t = origin.transform.translation;
                // A backwards bag clock starts a new map; do not merge different timelines.
                if (located_ && cloud->header.stamp < last_stamp_) { mapper_.clear(); newEpoch(); }
                const auto fuse_start = Clock::now();
                mapper_.integrate(points, rog_map::Vec3f(t.x, t.y, t.z));
                fuse_ms_ = ms(fuse_start);
                located_ = true; last_stamp_ = cloud->header.stamp; ++fused_;
                publish(last_stamp_);
                pipeline_ms_ = ms(received);
            } catch (const std::exception& error) {
                ++rejected_;
                ROS_WARN_THROTTLE(1.0, "Local map rejected scan: %s", error.what());
            }
        }
    }
    rog_map_msgs::LocalMap::ConstPtr snapshot() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_); return snapshot_;
    }
    void visualize(const ros::TimerEvent&) {
        if (!viz_pub_.getNumSubscribers()) return;
        const auto map = snapshot(); if (!map) return;
        visualization_msgs::MarkerArray out;
        visualization_msgs::Marker voxels;
        voxels.header = map->header; voxels.ns = "occupied"; voxels.id = 0;
        voxels.type = visualization_msgs::Marker::CUBE_LIST;
        voxels.action = visualization_msgs::Marker::ADD;
        voxels.pose.orientation.w = 1;
        voxels.scale.x = voxels.scale.y = voxels.scale.z = map->resolution;
        voxels.color.r = 0.2; voxels.color.g = 0.65; voxels.color.b = 1; voxels.color.a = 0.85;
        size_t i = 0;
        for (uint32_t z = 0; z < map->size[2]; ++z) for (uint32_t y = 0; y < map->size[1]; ++y)
            for (uint32_t x = 0; x < map->size[0]; ++x, ++i) if (rog_map_msgs::bit(map->occupied_bits, i)) {
                geometry_msgs::Point p;
                p.x = map->origin.x + (x + .5)*map->resolution;
                p.y = map->origin.y + (y + .5)*map->resolution;
                p.z = map->origin.z + (z + .5)*map->resolution;
                voxels.points.push_back(p);
            }
        if (voxels.points.empty()) voxels.action = visualization_msgs::Marker::DELETE;
        out.markers.push_back(std::move(voxels));
        visualization_msgs::Marker bound;
        bound.header = map->header; bound.ns = "window"; bound.id = 0;
        bound.type = visualization_msgs::Marker::LINE_LIST; bound.action = visualization_msgs::Marker::ADD;
        bound.pose.orientation.w = 1; bound.scale.x = 0.025;
        bound.color.r = 1; bound.color.g = 0.8; bound.color.a = 1;
        for (int a = 0; a < 3; ++a) for (int c = 0; c < 8; ++c) if (!(c & (1 << a))) {
            for (int e : {c, c | (1 << a)}) {
                geometry_msgs::Point p;
                p.x = map->origin.x + ((e & 1) ? map->size[0]*map->resolution : 0);
                p.y = map->origin.y + ((e & 2) ? map->size[1]*map->resolution : 0);
                p.z = map->origin.z + ((e & 4) ? map->size[2]*map->resolution : 0);
                bound.points.push_back(p);
            }
        }
        out.markers.push_back(bound); viz_pub_.publish(out);
    }
    void diagnose(const ros::WallTimerEvent&) {
        diagnostic_msgs::DiagnosticArray out; out.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status; status.name = ros::this_node::getName();
        const auto map = snapshot();
        status.level = map ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
        status.message = map ? "Local mapping" : "Waiting for a timestamped scan and TF";
        auto add = [&](const std::string& key, double value) {
            diagnostic_msgs::KeyValue item; item.key = key; item.value = std::to_string(value); status.values.push_back(item);
        };
        const double dt = std::chrono::duration<double>(Clock::now()-diag_at_).count(); diag_at_ = Clock::now();
        const uint64_t count = fused_.load(), input = received_.load();
        add("input_hz", (input-last_input_)/dt); last_input_ = input;
        add("fusion_hz", (count-last_fused_)/dt); last_fused_ = count;
        add("received", input); add("fused", count); add("published", published_);
        add("dropped", dropped_); add("rejected", rejected_); add("message_bytes", bytes_);
        add("tf_ms", tf_ms_); add("transform_ms", transform_ms_); add("fusion_ms", fuse_ms_);
        add("snapshot_ms", export_ms_); add("receive_to_publish_ms", pipeline_ms_);
        if (map) add("map_age_ms", (ros::Time::now()-map->header.stamp).toSec()*1000);
        out.status.push_back(status); diag_pub_.publish(out);
    }
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener listener_;
    rog_map::LocalMapper mapper_;
    std::string world_, sensor_; double tf_timeout_;
    ros::Subscriber cloud_sub_; ros::Publisher map_pub_, viz_pub_, diag_pub_;
    ros::ServiceServer reset_srv_; ros::Timer viz_timer_; ros::WallTimer diag_timer_;
    std::mutex mutex_, snapshot_mutex_; std::condition_variable cv_, reset_cv_; std::thread worker_;
    bool stop_ = false, reset_requested_ = false, located_ = false;
    sensor_msgs::PointCloud2::ConstPtr pending_;
    rog_map_msgs::LocalMap::ConstPtr snapshot_;
    Clock::time_point received_at_, diag_at_ = Clock::now();
    ros::Time last_stamp_; uint64_t epoch_ = 0, version_ = 0, last_fused_ = 0, last_input_ = 0;
    std::atomic<uint64_t> received_{0}, fused_{0}, published_{0}, dropped_{0}, rejected_{0}, bytes_{0};
    std::atomic<double> tf_ms_{0}, transform_ms_{0}, fuse_ms_{0}, export_ms_{0}, pipeline_ms_{0};
};
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "rog_map");
    try { LocalMapNode node; ros::spin(); }
    catch (const std::exception& error) { ROS_FATAL("ROG local mapper: %s", error.what()); return 1; }
    return 0;
}
