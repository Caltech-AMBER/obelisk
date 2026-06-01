#pragma once

#include "ray_caster_interface.h"

namespace obelisk {

/**
 * @brief Pinhole camera depth ray caster
 *
 * Creates rays matching a pinhole camera model, mimicking Isaac Lab's
 * PinholeCameraPatternCfg. Pixel coordinates are unprojected through the
 * inverse intrinsic matrix, then transformed from camera frame (x-right,
 * y-down, z-forward) to robotics frame (x-forward, y-left, z-up).
 * Uses full 3D rotation for both ray starts and directions.
 *
 * YAML Config:
 *   site_name: "camera_site"              # MuJoCo site name (required)
 *   geom_groups: [0]                      # Geom groups for collision (optional)
 *   max_distance: 0.0                     # Max ray distance in meters, 0 disables (optional)
 *   pattern:
 *     width: 30                           # Image width in pixels (required)
 *     height: 26                          # Image height in pixels (required)
 *     intrinsic_matrix: [fx, 0, cx,       # Row-major 3x3 intrinsic matrix (required)
 *                         0, fy, cy,
 *                         0,  0,  1]
 *     offset:                             # Offset transform applied in local frame (optional)
 *       pos: [0, 0, 0]                    # Position offset [x, y, z]
 *       rot: [1, 0, 0, 0]                # Rotation quaternion (w, x, y, z)
 */
class DepthInterface : public RayCasterInterface {
  public:
    explicit DepthInterface(const std::string& yaml_path) : RayCasterInterface(yaml_path) {
        setup_rays(get_config());
    }

    explicit DepthInterface(const YAML::Node& config) : RayCasterInterface(config) {
        setup_rays(get_config());
    }

    void compute_rays_world(const Matrix3d& site_xmat, const Vector3d& site_pos,
                            MatrixX3d& ray_starts_world,
                            MatrixX3d& ray_directions_world) const override {
        // Full 3D rotation for both starts and directions
        ray_starts_world = (site_xmat * ray_starts_local_.transpose()).transpose();
        ray_starts_world.rowwise() += site_pos.transpose();

        ray_directions_world = (site_xmat * ray_directions_local_.transpose()).transpose();
    }

    // Public accessors (overrides of virtual methods declared public in RayCasterInterface).
    float get_return(const std::array<double, 3> hit_point, const float dist) override {
        (void)hit_point;
        return dist;
    }

    int get_image_width() const override { return width_; }
    int get_image_height() const override { return height_; }
    Vector3d get_image_forward_local() const override { return image_forward_local_; }
    bool has_common_origin() const override { return true; }

  protected:
    void setup_rays(const YAML::Node& config) override {
        auto pattern = config["pattern"];
        if (!pattern) {
            throw std::runtime_error("Depth config missing 'pattern' section");
        }

        // Read image dimensions
        if (!pattern["width"]) {
            throw std::runtime_error("Depth config missing 'width'");
        }
        if (!pattern["height"]) {
            throw std::runtime_error("Depth config missing 'height'");
        }
        width_ = pattern["width"].as<int>();
        height_ = pattern["height"].as<int>();
        if (width_ <= 0 || height_ <= 0) {
            throw std::runtime_error("width and height must be > 0");
        }

        // Read intrinsic matrix (row-major, 9 elements: fx, 0, cx, 0, fy, cy, 0, 0, 1)
        if (!pattern["intrinsic_matrix"]) {
            throw std::runtime_error("Depth config missing 'intrinsic_matrix'");
        }
        auto K_vec = pattern["intrinsic_matrix"].as<std::vector<double>>();
        if (K_vec.size() != 9) {
            throw std::runtime_error("intrinsic_matrix must have exactly 9 elements");
        }

        // Build 3x3 intrinsic matrix and invert
        Eigen::Matrix3d K;
        K << K_vec[0], K_vec[1], K_vec[2],
             K_vec[3], K_vec[4], K_vec[5],
             K_vec[6], K_vec[7], K_vec[8];
        Eigen::Matrix3d K_inv = K.inverse();

        int num_rays = width_ * height_;
        ray_starts_local_.resize(num_rays, 3);
        ray_directions_local_.resize(num_rays, 3);

        // All rays start from origin in local frame
        ray_starts_local_.setZero();

        // Generate pixel grid with "xy" indexing (outer loop over y/height, inner over x/width)
        // and shift by +0.5 to center of pixel, matching Isaac Lab
        int idx = 0;
        for (int v = 0; v < height_; ++v) {
            for (int u = 0; u < width_; ++u) {
                // Pixel coordinates (centered)
                double px = u + 0.5;
                double py = v + 0.5;

                // Unproject through inverse intrinsic matrix
                // cam_dir = K_inv * [px, py, 1]^T
                Eigen::Vector3d pixel_h(px, py, 1.0);
                Eigen::Vector3d cam_dir = K_inv * pixel_h;

                // Transform from camera frame (x-right, y-down, z-forward)
                // to robotics frame (x-forward, y-left, z-up):
                // robot_x = cam_z, robot_y = -cam_x, robot_z = -cam_y
                Eigen::Vector3d robot_dir(cam_dir[2], -cam_dir[0], -cam_dir[1]);

                // Normalize to unit vector
                robot_dir.normalize();

                ray_directions_local_.row(idx) = robot_dir.transpose();
                idx++;
            }
        }

        // Optical axis: center pixel direction (before offset)
        {
            double cx = width_ / 2.0 + 0.5;
            double cy = height_ / 2.0 + 0.5;
            Eigen::Vector3d center_pixel(cx, cy, 1.0);
            Eigen::Vector3d cam_center = K_inv * center_pixel;
            image_forward_local_ = Vector3d(cam_center[2], -cam_center[0], -cam_center[1]);
            image_forward_local_.normalize();
        }

        // Apply offset transform
        Offset offset = parse_offset(pattern);
        apply_offset(offset);

        // Apply the same offset rotation to the optical axis
        image_forward_local_ = quat_apply(offset.rot, image_forward_local_);
        image_forward_local_.normalize();
    }

  private:
    int width_ = 0;
    int height_ = 0;
    Vector3d image_forward_local_ = Vector3d::Zero();
};

} // namespace obelisk
