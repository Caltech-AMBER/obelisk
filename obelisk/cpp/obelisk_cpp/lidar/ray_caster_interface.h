#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <mujoco/mujoco.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace obelisk {

/**
 * @brief Apply quaternion rotation to a vector
 * @param quat Quaternion (w, x, y, z)
 * @param vec 3D vector to rotate
 * @return Rotated vector
 */
inline Eigen::Vector3d quat_apply(const Eigen::Quaterniond& quat, const Eigen::Vector3d& vec) {
    return quat * vec;
}

/**
 * @brief Abstract base class for ray casting sensors (LIDAR, height scan, etc.)
 *
 * This interface provides a common structure for computing ray origins and directions
 * in both local and world frames. Concrete implementations define the ray pattern
 * (grid, spherical, etc.) and transformation behavior.
 *
 * Usage:
 *   1. Construct with path to YAML config file
 *   2. Call compute_rays_world() with site pose to get world-frame rays
 *   3. Use rays with mj_ray() for raycasting
 */
class RayCasterInterface {
  public:
    using Matrix3d = Eigen::Matrix3d;
    using Vector3d = Eigen::Vector3d;
    using MatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;

    /**
     * @brief Construct from YAML config file path
     * @param yaml_path Path to YAML configuration file
     * @throws std::runtime_error if file not found or missing required fields
     */
    explicit RayCasterInterface(const std::string& yaml_path) {
        if (!std::filesystem::exists(yaml_path)) {
            throw std::runtime_error("Ray caster config file not found: " + yaml_path);
        }
        load_common(YAML::LoadFile(yaml_path));
    }

    /**
     * @brief Construct from a parsed YAML node (inline config; no file I/O).
     * @throws std::runtime_error if the node is missing required fields
     */
    explicit RayCasterInterface(const YAML::Node& config) { load_common(config); }

    virtual ~RayCasterInterface() = default;

    // Disable copy (YAML::Node has complex copy semantics)
    RayCasterInterface(const RayCasterInterface&) = delete;
    RayCasterInterface& operator=(const RayCasterInterface&) = delete;

    // Allow move
    RayCasterInterface(RayCasterInterface&&) = default;
    RayCasterInterface& operator=(RayCasterInterface&&) = default;

    /**
     * @brief Get the MuJoCo site name for ray origin reference
     */
    const std::string& get_site() const { return site_name_; }

    /**
     * @brief Get number of rays in the pattern
     */
    int get_num_rays() const { return static_cast<int>(ray_starts_local_.rows()); }

    /**
     * @brief Get ray starts in local frame (read-only)
     */
    const MatrixX3d& get_ray_starts_local() const { return ray_starts_local_; }

    /**
     * @brief Get ray directions in local frame (read-only)
     */
    const MatrixX3d& get_ray_directions_local() const { return ray_directions_local_; }

    /**
     * @brief Get geom group mask for mj_ray
     */
    const mjtByte* get_geom_group_mask() const { return geom_group_mask_; }

    /**
     * @brief Get the frame_id string for published messages
     */
    const std::string& get_frame() const { return frame_; }

    /**
     * @brief Max ray distance in meters. 0.0 means disabled (no limit).
     */
    double get_max_distance() const { return max_distance_; }

    /**
     * @brief Clamp a raycast distance by max_distance.
     *
     * If max_distance is configured (>0) and the hit is farther than the limit,
     * returns -1 (the mj_ray no-hit sentinel) so callers can treat it as a miss.
     * Otherwise returns dist unchanged (including the existing -1 no-hit case).
     */
    double apply_max_distance(double dist) const {
        if (max_distance_ > 0.0 && dist > max_distance_) {
            return -1.0;
        }
        return dist;
    }

    /**
     * @brief Compute ray starts and directions in world frame
     * @param site_xmat 3x3 rotation matrix from site orientation
     * @param site_pos 3D position of site in world frame
     * @param[out] ray_starts_world Output matrix of ray start positions (Nx3)
     * @param[out] ray_directions_world Output matrix of ray directions (Nx3)
     */
    virtual void compute_rays_world(const Matrix3d& site_xmat, const Vector3d& site_pos,
                                    MatrixX3d& ray_starts_world,
                                    MatrixX3d& ray_directions_world) const = 0;

    /**
     * @brief Construct the proper return type for the ray caster
     * @param hit_point The hit point in 3D
     * @param dist The distance to the hit point
     */
    virtual float get_return(const std::array<double, 3> hit_point, const float dist) = 0;

    virtual int get_image_width() const { return -1; }
    virtual int get_image_height() const { return -1; }
    virtual Vector3d get_image_forward_local() const { return Vector3d::Zero(); }

  protected:
    /**
     * @brief Offset transform (position and optional rotation)
     */
    struct Offset {
        Vector3d pos = Vector3d::Zero();
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity(); // (w, x, y, z)
    };

    /**
     * @brief Setup ray pattern from YAML config (called by derived class constructor)
     * @param config YAML node containing pattern-specific configuration
     */
    virtual void setup_rays(const YAML::Node& config) = 0;

    /**
     * @brief Get the loaded config for derived classes
     */
    const YAML::Node& get_config() const { return config_; }

    /**
     * @brief Parse offset from YAML pattern config
     * @param pattern YAML node containing the pattern section
     * @return Offset struct with position and rotation
     */
    Offset parse_offset(const YAML::Node& pattern) const {
        Offset offset;

        if (pattern["offset"]) {
            auto offset_node = pattern["offset"];

            // Read position offset
            if (offset_node["pos"]) {
                auto pos_vec = offset_node["pos"].as<std::vector<double>>();
                if (pos_vec.size() == 3) {
                    offset.pos = Vector3d(pos_vec[0], pos_vec[1], pos_vec[2]);
                }
            }

            // Read rotation offset (quaternion: w, x, y, z)
            if (offset_node["rot"]) {
                auto rot_vec = offset_node["rot"].as<std::vector<double>>();
                if (rot_vec.size() == 4) {
                    // YAML format: (w, x, y, z)
                    offset.rot = Eigen::Quaterniond(rot_vec[0], rot_vec[1], rot_vec[2], rot_vec[3]);
                    offset.rot.normalize();
                }
            }
        }

        return offset;
    }

    /**
     * @brief Apply offset to ray starts and directions
     * @param offset The offset to apply
     *
     * Applies rotation to directions, then adds position to starts
     */
    void apply_offset(const Offset& offset) {
        // Apply rotation to directions
        for (int i = 0; i < ray_directions_local_.rows(); ++i) {
            Vector3d dir = ray_directions_local_.row(i).transpose();
            Vector3d rotated_dir = quat_apply(offset.rot, dir);
            ray_directions_local_.row(i) = rotated_dir.transpose();
        }

        // Apply position offset to starts
        ray_starts_local_.rowwise() += offset.pos.transpose();
    }

    // Local-frame ray data (set by derived classes in setup_rays)
    MatrixX3d ray_starts_local_;     // Nx3 matrix of ray start positions
    MatrixX3d ray_directions_local_; // Nx3 matrix of ray directions (unit vectors)

  private:
    /**
     * @brief Common initialization: validate and pull top-level fields out of the YAML node.
     */
    void load_common(const YAML::Node& config) {
        config_ = config;

        // Read site name (required)
        if (config_["site_name"]) {
            site_name_ = config_["site_name"].as<std::string>();
        } else {
            throw std::runtime_error("site_name is required in ray caster config");
        }

        // Read geom groups for mj_ray filtering (optional, default: group 0 only)
        std::fill(std::begin(geom_group_mask_), std::end(geom_group_mask_), 0);
        if (config_["geom_groups"]) {
            auto groups = config_["geom_groups"].as<std::vector<int>>();
            for (int g : groups) {
                if (g >= 0 && g < mjNGROUP) {
                    geom_group_mask_[g] = 1;
                }
            }
        } else {
            geom_group_mask_[0] = 1;
        }

        // Read frame (optional, default: "world")
        if (config_["frame"]) {
            frame_ = config_["frame"].as<std::string>();
        }

        // Read max ray distance (optional, default: 0.0 meaning "disabled")
        // Hits beyond this distance are treated as no-hit (apply_max_distance returns -1).
        if (config_["max_distance"]) {
            max_distance_ = config_["max_distance"].as<double>();
            if (max_distance_ < 0.0) {
                throw std::runtime_error("max_distance must be >= 0");
            }
        }
    }

    YAML::Node config_;
    std::string site_name_;
    mjtByte geom_group_mask_[mjNGROUP];
    std::string frame_ = "world";
    double max_distance_ = 0.0; // 0 => disabled
};

} // namespace obelisk
