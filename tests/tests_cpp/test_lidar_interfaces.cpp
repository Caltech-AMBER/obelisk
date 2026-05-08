// Tests for the ray-caster interfaces under obelisk/cpp/obelisk_cpp/lidar/.
//
// These cover the configuration plumbing of LidarInterface, HeightScanInterface, and DepthInterface
// without exercising MuJoCo (mj_ray etc.). They check that loading a config yields the expected ray
// pattern dimensions, site/frame metadata, and return-value semantics.
//
// Both the legacy file-path constructor and the inline YAML::Node constructor are exercised, so
// these tests guard against regression of the inline-config refactor.

#include <catch2/catch_test_macros.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "depth_interface.h"
#include "height_scan_interface.h"
#include "lidar_interface.h"

namespace {

// Helper: write a YAML string to a unique temp file and return the path. Caller is responsible for
// removing the file after the test.
std::filesystem::path WriteTempYaml(const std::string& contents, const std::string& tag) {
    auto path = std::filesystem::temp_directory_path() /
                ("obk_lidar_test_" + tag + "_" + std::to_string(std::rand()) + ".yaml");
    std::ofstream out(path);
    REQUIRE(out.is_open());
    out << contents;
    out.close();
    return path;
}

struct TempYaml {
    std::filesystem::path path;
    ~TempYaml() { std::filesystem::remove(path); }
};

} // namespace

// ---------------------------------------------------------------------------- //
// LidarInterface                                                               //
// ---------------------------------------------------------------------------- //

constexpr const char* kLidarYaml = R"(
site_name: head_lidar_site
geom_groups: [2]
frame: body
max_distance: 0.0
pattern:
  type: lidar_scan
  vertical_fov_range: [0.0, 51.0]
  horizontal_fov_range: [0.0, 360.0]
  channels: 51
  horizontal_res: 1.0
  offset:
    pos: [0.0, 0.0, 0.0]
    rot: [1.0, 0.0, 0.0, 0.0]
)";

TEST_CASE("LidarInterface: load from file path", "[lidar][config_path]") {
    TempYaml tmp{WriteTempYaml(kLidarYaml, "lidar")};

    obelisk::LidarInterface lidar(tmp.path.string());

    CHECK(lidar.get_site() == "head_lidar_site");
    CHECK(lidar.get_frame() == "body");
    CHECK(lidar.get_max_distance() == 0.0);
    // 51 vertical channels x horizontal samples. With horizontal_res=1° over [0,360°], the loop
    // emits 360 samples and drops the last one (wraparound at 0/360), leaving 359.
    CHECK(lidar.get_image_height() == 51);
    CHECK(lidar.get_image_width() == 359);
    CHECK(lidar.get_num_rays() == 51 * 359);
}

TEST_CASE("LidarInterface: load from YAML::Node (inline)", "[lidar][inline]") {
    YAML::Node node = YAML::Load(kLidarYaml);

    obelisk::LidarInterface lidar(node);

    CHECK(lidar.get_site() == "head_lidar_site");
    CHECK(lidar.get_frame() == "body");
    CHECK(lidar.get_image_height() == 51);
    CHECK(lidar.get_image_width() == 359);
    CHECK(lidar.get_num_rays() == 51 * 359);
}

TEST_CASE("LidarInterface: get_return passes distance through", "[lidar]") {
    YAML::Node node = YAML::Load(kLidarYaml);
    obelisk::LidarInterface lidar(node);

    std::array<double, 3> hit{1.2, 3.4, 5.6};
    CHECK(lidar.get_return(hit, 7.5f) == 7.5f);
    CHECK(lidar.get_return(hit, -1.0f) == -1.0f);
}

TEST_CASE("LidarInterface: missing required pattern fields throws", "[lidar][errors]") {
    YAML::Node node = YAML::Load(R"(
site_name: lidar_site
pattern:
  type: lidar_scan
  vertical_fov_range: [0.0, 30.0]
  channels: 4
  horizontal_res: 5.0
)"); // missing horizontal_fov_range

    CHECK_THROWS(obelisk::LidarInterface{node});
}

// ---------------------------------------------------------------------------- //
// HeightScanInterface                                                          //
// ---------------------------------------------------------------------------- //

constexpr const char* kHeightScanYaml = R"(
site_name: pelvis_mocap_site
geom_groups: [2]
pattern:
  type: height_scan
  ordering: xy
  resolution: 0.1
  size: [1.6, 0.8]
  offset:
    pos: [0.08, 0.0, 20.0]
)";

TEST_CASE("HeightScanInterface: load from file path", "[height_scan][config_path]") {
    TempYaml tmp{WriteTempYaml(kHeightScanYaml, "height")};

    obelisk::HeightScanInterface hs(tmp.path.string());

    CHECK(hs.get_site() == "pelvis_mocap_site");
    CHECK(hs.get_frame() == "world"); // default when frame not specified
    // 1.6 / 0.1 + 1 = 17 columns; 0.8 / 0.1 + 1 = 9 rows
    CHECK(hs.get_num_rays() == 17 * 9);
}

TEST_CASE("HeightScanInterface: load from YAML::Node (inline)", "[height_scan][inline]") {
    YAML::Node node = YAML::Load(kHeightScanYaml);

    obelisk::HeightScanInterface hs(node);

    CHECK(hs.get_site() == "pelvis_mocap_site");
    CHECK(hs.get_num_rays() == 17 * 9);
}

TEST_CASE("HeightScanInterface: get_return returns hit-point z", "[height_scan]") {
    YAML::Node node = YAML::Load(kHeightScanYaml);
    obelisk::HeightScanInterface hs(node);

    std::array<double, 3> hit{0.1, 0.2, 4.25};
    CHECK(hs.get_return(hit, 99.0f) == 4.25f); // hit_point[2] returned, dist ignored
}

TEST_CASE("HeightScanInterface: invalid ordering throws", "[height_scan][errors]") {
    YAML::Node node = YAML::Load(R"(
site_name: hs
pattern:
  ordering: zigzag
  resolution: 0.1
  size: [1.0, 1.0]
)");

    CHECK_THROWS(obelisk::HeightScanInterface{node});
}

// ---------------------------------------------------------------------------- //
// DepthInterface                                                               //
// ---------------------------------------------------------------------------- //

constexpr const char* kDepthYaml = R"(
site_name: pelvis_mocap_site
geom_groups: [0, 1, 2]
frame: world
pattern:
  type: depth_camera
  width: 30
  height: 26
  intrinsic_matrix: [25.9928, 0.0, 14.9936, 0.0, 27.8703, 12.9966, 0.0, 0.0, 1.0]
  offset:
    pos: [0.153246, 0.0, 0.106799]
    rot: [0.843391, 0.0, 0.537300, 0.0]
)";

TEST_CASE("DepthInterface: load from file path", "[depth][config_path]") {
    TempYaml tmp{WriteTempYaml(kDepthYaml, "depth")};

    obelisk::DepthInterface depth(tmp.path.string());

    CHECK(depth.get_site() == "pelvis_mocap_site");
    CHECK(depth.get_frame() == "world");
    CHECK(depth.get_image_width() == 30);
    CHECK(depth.get_image_height() == 26);
    CHECK(depth.get_num_rays() == 30 * 26);
}

TEST_CASE("DepthInterface: load from YAML::Node (inline)", "[depth][inline]") {
    YAML::Node node = YAML::Load(kDepthYaml);

    obelisk::DepthInterface depth(node);

    CHECK(depth.get_image_width() == 30);
    CHECK(depth.get_image_height() == 26);
    CHECK(depth.get_num_rays() == 30 * 26);
}

TEST_CASE("DepthInterface: missing intrinsic_matrix throws", "[depth][errors]") {
    YAML::Node node = YAML::Load(R"(
site_name: cam
pattern:
  width: 4
  height: 3
)");

    CHECK_THROWS(obelisk::DepthInterface{node});
}

// ---------------------------------------------------------------------------- //
// Common RayCasterInterface behavior                                           //
// ---------------------------------------------------------------------------- //

TEST_CASE("RayCasterInterface: max_distance clamps long hits", "[ray_caster][max_distance]") {
    YAML::Node node = YAML::Load(R"(
site_name: lidar_site
max_distance: 5.0
pattern:
  type: lidar_scan
  vertical_fov_range: [0.0, 0.0]
  horizontal_fov_range: [0.0, 360.0]
  channels: 1
  horizontal_res: 90.0
)");

    obelisk::LidarInterface lidar(node);

    CHECK(lidar.get_max_distance() == 5.0);
    CHECK(lidar.apply_max_distance(3.0) == 3.0);    // within range
    CHECK(lidar.apply_max_distance(10.0) == -1.0);  // beyond range -> sentinel
    CHECK(lidar.apply_max_distance(-1.0) == -1.0);  // existing no-hit unchanged
}

TEST_CASE("RayCasterInterface: missing site_name throws", "[ray_caster][errors]") {
    YAML::Node node = YAML::Load(R"(
pattern:
  type: lidar_scan
  vertical_fov_range: [0.0, 30.0]
  horizontal_fov_range: [0.0, 90.0]
  channels: 4
  horizontal_res: 5.0
)");

    CHECK_THROWS(obelisk::LidarInterface{node});
}

TEST_CASE("RayCasterInterface: file path constructor errors on missing file", "[ray_caster][errors]") {
    CHECK_THROWS(obelisk::LidarInterface{std::string("/nonexistent/path/to/lidar.yaml")});
}
