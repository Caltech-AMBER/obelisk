#pragma once

#include "ray_caster_interface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace obelisk {

/**
 * @brief LIDAR ray caster using a spherical pattern
 *
 * Creates rays in a spherical pattern based on vertical/horizontal FOV.
 * Uses full 3D rotation for both ray starts and directions.
 * Suitable for simulating 3D LIDAR sensors like Velodyne, Ouster, etc.
 *
 * YAML Config:
 *   site_name: "lidar_site"            # MuJoCo site name (required)
 *   geom_groups: [0]                   # Geom groups for collision (optional)
 *   pattern:
 *     vertical_fov_range: [-15, 15]    # [min, max] degrees (required)
 *     horizontal_fov_range: [0, 360]   # [min, max] degrees (required)
 *     channels: 16                     # Number of vertical channels (required)
 *     horizontal_res: 1.0              # Degrees per horizontal step (required)
 *     offset:                          # Offset transform applied in local frame (optional)
 *       pos: [0, 0, 0]                 # Position offset [x, y, z]
 *       rot: [1, 0, 0, 0]              # Rotation quaternion (w, x, y, z)
 */
class LidarInterface : public RayCasterInterface {
  public:
    explicit LidarInterface(const std::string& yaml_path) : RayCasterInterface(yaml_path) {
        setup_rays(get_config());
    }

    void compute_rays_world(const Matrix3d& site_xmat, const Vector3d& site_pos,
                            MatrixX3d& ray_starts_world,
                            MatrixX3d& ray_directions_world) const override {
        // Full 3D rotation for both starts and directions
        // ray_starts_local_ is Nx3, site_xmat is 3x3
        // world = R * local^T, then transpose back to Nx3

        ray_starts_world = (site_xmat * ray_starts_local_.transpose()).transpose();
        ray_starts_world.rowwise() += site_pos.transpose();

        ray_directions_world = (site_xmat * ray_directions_local_.transpose()).transpose();
    }

  protected:
    void setup_rays(const YAML::Node& config) override {
        auto pattern = config["pattern"];
        if (!pattern) {
            throw std::runtime_error("Lidar config missing 'pattern' section");
        }

        // Read vertical FOV range [min_deg, max_deg]
        if (!pattern["vertical_fov_range"]) {
            throw std::runtime_error("Lidar config missing 'vertical_fov_range'");
        }
        auto vert_fov = pattern["vertical_fov_range"].as<std::vector<double>>();
        if (vert_fov.size() != 2) {
            throw std::runtime_error("vertical_fov_range must have 2 elements [min, max]");
        }
        double vert_min_deg = vert_fov[0];
        double vert_max_deg = vert_fov[1];

        // Read horizontal FOV range [min_deg, max_deg]
        if (!pattern["horizontal_fov_range"]) {
            throw std::runtime_error("Lidar config missing 'horizontal_fov_range'");
        }
        auto horz_fov = pattern["horizontal_fov_range"].as<std::vector<double>>();
        if (horz_fov.size() != 2) {
            throw std::runtime_error("horizontal_fov_range must have 2 elements [min, max]");
        }
        double horz_min_deg = horz_fov[0];
        double horz_max_deg = horz_fov[1];

        // Read number of vertical channels
        if (!pattern["channels"]) {
            throw std::runtime_error("Lidar config missing 'channels'");
        }
        int channels = pattern["channels"].as<int>();
        if (channels <= 0) {
            throw std::runtime_error("channels must be > 0");
        }

        // Read horizontal resolution (degrees per step)
        if (!pattern["horizontal_res"]) {
            throw std::runtime_error("Lidar config missing 'horizontal_res'");
        }
        double horz_res_deg = pattern["horizontal_res"].as<double>();
        if (horz_res_deg <= 0) {
            throw std::runtime_error("horizontal_res must be > 0");
        }

        // Compute vertical angles (in radians)
        std::vector<double> vert_angles_rad;
        if (channels == 1) {
            vert_angles_rad.push_back(vert_min_deg * M_PI / 180.0);
        } else {
            for (int i = 0; i < channels; ++i) {
                double angle_deg =
                    vert_min_deg + i * (vert_max_deg - vert_min_deg) / (channels - 1);
                vert_angles_rad.push_back(angle_deg * M_PI / 180.0);
            }
        }

        // Compute horizontal angles
        double horz_span = horz_max_deg - horz_min_deg;
        int num_horizontal =
            static_cast<int>(std::ceil(horz_span / horz_res_deg)); // Isaac doesn't add +1 for endpoints...

        // Handle 360-degree wraparound: exclude last point to avoid overlap
        bool is_full_rotation = std::abs(std::abs(horz_span) - 360.0) < 1e-6;

        std::vector<double> horz_angles_rad;
        for (int i = 0; i < num_horizontal; ++i) {
            // Skip last point for full rotation to avoid duplicate at 0/360
            if (is_full_rotation && i == num_horizontal - 1) {
                continue;
            }
            double angle_deg;
            if (num_horizontal == 1) {
                angle_deg = horz_min_deg;
            } else {
                angle_deg = horz_min_deg + i * (horz_max_deg - horz_min_deg) / (num_horizontal - 1);
            }
            horz_angles_rad.push_back(angle_deg * M_PI / 180.0);
        }

        // Create meshgrid and compute ray directions (spherical to Cartesian)
        int nv = static_cast<int>(vert_angles_rad.size());
        int nh = static_cast<int>(horz_angles_rad.size());
        int num_rays = nv * nh;

        ray_starts_local_.resize(num_rays, 3);
        ray_directions_local_.resize(num_rays, 3);

        // All rays start from origin in local frame
        ray_starts_local_.setZero();

        nv_ = nv;
        nh_ = nh;

        // Meshgrid with indexing="ij" (outer: vertical, inner: horizontal)
        int idx = 0;
        for (int vi = 0; vi < nv; ++vi) {
            double v_angle = vert_angles_rad[vi];
            double cv = std::cos(v_angle);
            double sv = std::sin(v_angle);

            for (int hi = 0; hi < nh; ++hi) {
                double h_angle = horz_angles_rad[hi];
                double ch = std::cos(h_angle);
                double sh = std::sin(h_angle);

                // Spherical to Cartesian (Z up convention)
                // x = cos(elevation) * cos(azimuth)
                // y = cos(elevation) * sin(azimuth)
                // z = sin(elevation)
                ray_directions_local_(idx, 0) = cv * ch;
                ray_directions_local_(idx, 1) = cv * sh;
                ray_directions_local_(idx, 2) = sv;
                idx++;
            }
        }

        // Apply offset transform (rotation to directions, position to starts)
        Offset offset = parse_offset(pattern);
        apply_offset(offset);
    }

    float get_return(const std::array<double, 3> hit_point, const float dist) override {
        (void)hit_point;
        return dist;
    }

    int get_image_width() const override { return nh_; }
    int get_image_height() const override { return nv_; }

  private:
    int nv_ = 0;
    int nh_ = 0;
};

} // namespace obelisk
