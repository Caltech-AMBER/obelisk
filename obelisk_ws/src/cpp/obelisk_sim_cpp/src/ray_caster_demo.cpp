#include "height_scan_interface.h"
#include "lidar_interface.h"

#include <iostream>
#include <iomanip>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: ray_caster_demo <height_scan_yaml> <lidar_yaml>\n"
                  << "  Each argument is a path to a YAML config (same schema as the inline\n"
                  << "  scan_config: blocks under sensor_settings in the obelisk launch YAMLs).\n";
        return 1;
    }
    std::string height_scan_config = argv[1];
    std::string lidar_config       = argv[2];

    std::cout << "=== Height Scan Interface ===" << std::endl;
    {
        obelisk::HeightScanInterface scanner(height_scan_config);

        std::cout << "Site name: " << scanner.get_site() << std::endl;
        std::cout << "Num rays: " << scanner.get_num_rays() << std::endl;

        const auto& starts = scanner.get_ray_starts_local();
        const auto& dirs = scanner.get_ray_directions_local();

        std::cout << "\nFirst 5 rays (local frame):" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < std::min(5, scanner.get_num_rays()); ++i) {
            std::cout << "  Ray " << i << ": start=("
                        << starts(i, 0) << ", " << starts(i, 1) << ", " << starts(i, 2)
                        << ") dir=("
                        << dirs(i, 0) << ", " << dirs(i, 1) << ", " << dirs(i, 2)
                        << ")" << std::endl;
        }

        // Test world transform with identity
        Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(1.0, 2.0, 3.0);
        Eigen::Matrix<double, Eigen::Dynamic, 3> starts_w, dirs_w;
        scanner.compute_rays_world(identity, pos, starts_w, dirs_w);

        std::cout << "\nFirst 5 rays (world frame, pos=[1,2,3], identity rotation):" << std::endl;
        for (int i = 0; i < std::min(5, scanner.get_num_rays()); ++i) {
            std::cout << "  Ray " << i << ": start=("
                        << starts_w(i, 0) << ", " << starts_w(i, 1) << ", " << starts_w(i, 2)
                        << ") dir=("
                        << dirs_w(i, 0) << ", " << dirs_w(i, 1) << ", " << dirs_w(i, 2)
                        << ")" << std::endl;
        }
    }
    

    std::cout << "\n=== Lidar Interface ===" << std::endl;
    {
        obelisk::LidarInterface lidar(lidar_config);

        std::cout << "Site name: " << lidar.get_site() << std::endl;
        std::cout << "Num rays: " << lidar.get_num_rays() << std::endl;

        const auto& starts = lidar.get_ray_starts_local();
        const auto& dirs = lidar.get_ray_directions_local();

        std::cout << "\nFirst 5 rays (local frame):" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < std::min(5, lidar.get_num_rays()); ++i) {
            std::cout << "  Ray " << i << ": start=("
                        << starts(i, 0) << ", " << starts(i, 1) << ", " << starts(i, 2)
                        << ") dir=("
                        << dirs(i, 0) << ", " << dirs(i, 1) << ", " << dirs(i, 2)
                        << ")" << std::endl;
        }

        // Test world transform with 90-degree yaw
        Eigen::Matrix3d Rz_90;
        Rz_90 << 0, -1, 0,
                    1,  0, 0,
                    0,  0, 1;
        Eigen::Vector3d pos(0.0, 0.0, 1.0);
        Eigen::Matrix<double, Eigen::Dynamic, 3> starts_w, dirs_w;
        lidar.compute_rays_world(Rz_90, pos, starts_w, dirs_w);

        std::cout << "\nFirst 5 rays (world frame, pos=[0,0,1], 90deg yaw):" << std::endl;
        for (int i = 0; i < std::min(5, lidar.get_num_rays()); ++i) {
            std::cout << "  Ray " << i << ": start=("
                        << starts_w(i, 0) << ", " << starts_w(i, 1) << ", " << starts_w(i, 2)
                        << ") dir=("
                        << dirs_w(i, 0) << ", " << dirs_w(i, 1) << ", " << dirs_w(i, 2)
                        << ")" << std::endl;
        }
    }
    
    return 0;
}
