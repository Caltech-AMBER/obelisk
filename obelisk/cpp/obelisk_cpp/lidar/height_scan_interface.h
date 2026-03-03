#pragma once

#include "ray_caster_interface.h"

namespace obelisk {

/**
 * @brief Height scan ray caster using a grid pattern
 *
 * Creates a 2D grid of rays pointing in a fixed direction (typically downward).
 * Uses yaw-only rotation for the ray starts, but does NOT rotate ray directions.
 * This is suitable for terrain height mapping on legged robots.
 *
 * YAML Config:
 *   site_name: "height_scan_site"   # MuJoCo site name (required)
 *   geom_groups: [0]                # Geom groups for collision (optional)
 *   pattern:
 *     ordering: "xy"                # Grid ordering: "xy" or "yx" (optional, default: "xy")
 *     resolution: 0.05              # Grid step size in meters (required)
 *     size: [1.0, 1.0]              # [width, height] in meters (required)
 *     direction: [0, 0, -1]         # Ray direction in local frame (optional, default: down)
 *     offset:                       # Offset transform applied in local frame (optional)
 *       pos: [0, 0, 0]              # Position offset [x, y, z]
 *       rot: [1, 0, 0, 0]           # Rotation quaternion (w, x, y, z)
 */
class HeightScanInterface : public RayCasterInterface {
  public:
    explicit HeightScanInterface(const std::string& yaml_path) : RayCasterInterface(yaml_path) {
        setup_rays(get_config());
    }

    void compute_rays_world(const Matrix3d& site_xmat, const Vector3d& site_pos,
                            MatrixX3d& ray_starts_world,
                            MatrixX3d& ray_directions_world) const override {
        // Extract yaw from rotation matrix
        // site_xmat is column-major in Eigen: R(row, col)
        // yaw = atan2(R[1,0], R[0,0])
        double yaw = std::atan2(site_xmat(1, 0), site_xmat(0, 0));
        double c = std::cos(yaw);
        double s = std::sin(yaw);

        // Yaw-only rotation matrix (rotation about Z axis)
        Matrix3d Rz;
        Rz << c, -s, 0.0,
              s, c, 0.0,
              0.0, 0.0, 1.0;

        // Transform ray starts: Rz * starts^T, then transpose back to Nx3
        ray_starts_world = (Rz * ray_starts_local_.transpose()).transpose();
        ray_starts_world.rowwise() += site_pos.transpose();

        // Directions are NOT rotated for height scan (always point in fixed world direction)
        ray_directions_world = ray_directions_local_;
    }

  protected:
    void setup_rays(const YAML::Node& config) override {
        auto pattern = config["pattern"];
        if (!pattern) {
            throw std::runtime_error("HeightScan config missing 'pattern' section");
        }

        // Read ordering: "xy" or "yx" (affects meshgrid indexing)
        std::string ordering = "xy";
        if (pattern["ordering"]) {
            ordering = pattern["ordering"].as<std::string>();
            if (ordering != "xy" && ordering != "yx") {
                throw std::runtime_error("ordering must be 'xy' or 'yx', got: " + ordering);
            }
        }

        // Read resolution (step size in meters)
        if (!pattern["resolution"]) {
            throw std::runtime_error("HeightScan config missing 'resolution'");
        }
        double resolution = pattern["resolution"].as<double>();
        if (resolution <= 0) {
            throw std::runtime_error("resolution must be > 0");
        }

        // Read size [width, height] in meters
        if (!pattern["size"]) {
            throw std::runtime_error("HeightScan config missing 'size'");
        }
        auto size_vec = pattern["size"].as<std::vector<double>>();
        if (size_vec.size() != 2) {
            throw std::runtime_error("size must have exactly 2 elements [width, height]");
        }
        double width = size_vec[0];
        double height = size_vec[1];

        // Read direction (default: downward)
        Vector3d direction(0.0, 0.0, -1.0);
        if (pattern["direction"]) {
            auto dir_vec = pattern["direction"].as<std::vector<double>>();
            if (dir_vec.size() == 3) {
                direction = Vector3d(dir_vec[0], dir_vec[1], dir_vec[2]);
                double norm = direction.norm();
                if (norm > 1e-9) {
                    direction /= norm;
                } else {
                    throw std::runtime_error("direction must have positive norm");
                }
            }
        }

        // Generate grid points
        // x: from -width/2 to +width/2, step = resolution
        // y: from -height/2 to +height/2, step = resolution
        std::vector<double> x_vals, y_vals;
        for (double x = -width / 2.0; x <= width / 2.0 + 1e-9; x += resolution) {
            x_vals.push_back(x);
        }
        for (double y = -height / 2.0; y <= height / 2.0 + 1e-9; y += resolution) {
            y_vals.push_back(y);
        }

        int nx = static_cast<int>(x_vals.size());
        int ny = static_cast<int>(y_vals.size());
        int num_rays = nx * ny;

        ray_starts_local_.resize(num_rays, 3);
        ray_directions_local_.resize(num_rays, 3);

        // Create meshgrid and flatten
        // ordering == "xy": numpy indexing='xy', outer loop over y, inner over x
        // ordering == "yx": numpy indexing='ij', outer loop over x, inner over y
        int idx = 0;
        if (ordering == "xy") {
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    ray_starts_local_(idx, 0) = x_vals[i];
                    ray_starts_local_(idx, 1) = y_vals[j];
                    ray_starts_local_(idx, 2) = 0.0;
                    idx++;
                }
            }
        } else { // "yx" (or "ij" indexing)
            for (int i = 0; i < nx; ++i) {
                for (int j = 0; j < ny; ++j) {
                    ray_starts_local_(idx, 0) = x_vals[i];
                    ray_starts_local_(idx, 1) = y_vals[j];
                    ray_starts_local_(idx, 2) = 0.0;
                    idx++;
                }
            }
        }

        // All rays have the same direction
        for (int i = 0; i < num_rays; ++i) {
            ray_directions_local_.row(i) = direction.transpose();
        }

        // Apply offset transform (rotation to directions, position to starts)
        Offset offset = parse_offset(pattern);
        apply_offset(offset);
    }

    float get_return(const std::array<double, 3> hit_point, const float dist) override {
        (void)dist;
        return float(hit_point[2]);
    }
};

} // namespace obelisk
