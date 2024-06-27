// test_obelisk_node.cpp

#include "obelisk_node.h"
#include <catch2/catch_test_macros.hpp>

class TestObeliskNode : public obelisk::ObeliskNode {
  public:
    TestObeliskNode() : ObeliskNode("test_node") {}

    // Expose protected methods for testing
    using ObeliskNode::GetCallbackGroupName;
    using ObeliskNode::GetHistoryDepth;
    using ObeliskNode::GetIsObeliskMsg;
    using ObeliskNode::GetMessageName;
    using ObeliskNode::GetPeriod;
    using ObeliskNode::GetTopic;
    using ObeliskNode::ParseCallbackGroupConfig;
    using ObeliskNode::ParseConfigStr;
};

TEST_CASE("ParseConfigStr", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;

    SECTION("Valid config string") {
        auto result = node.ParseConfigStr("topic:/test_topic,history_depth:10,non_obelisk:false");
        REQUIRE(result.size() == 3);
        REQUIRE(result["topic"] == "/test_topic");
        REQUIRE(result["history_depth"] == "10");
        REQUIRE(result["non_obelisk"] == "false");
    }

    SECTION("Invalid config string") { REQUIRE_THROWS_AS(node.ParseConfigStr("invalid_string"), std::runtime_error); }

    rclcpp::shutdown();
}

TEST_CASE("GetTopic", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"topic", "/test_topic"}};

    REQUIRE(node.GetTopic(config_map) == "/test_topic");

    config_map.clear();
    REQUIRE_THROWS_AS(node.GetTopic(config_map), std::runtime_error);

    rclcpp::shutdown();
}

TEST_CASE("GetHistoryDepth", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"history_depth", "20"}};

    REQUIRE(node.GetHistoryDepth(config_map) == 20);

    config_map.clear();
    REQUIRE(node.GetHistoryDepth(config_map) == 10); // Default value

    rclcpp::shutdown();
}

TEST_CASE("GetIsObeliskMsg", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"non_obelisk", "true"}};

    REQUIRE(node.GetIsObeliskMsg(config_map) == false);

    config_map["non_obelisk"] = "false";
    REQUIRE(node.GetIsObeliskMsg(config_map) == true);

    config_map.clear();
    REQUIRE(node.GetIsObeliskMsg(config_map) == true); // Default value

    rclcpp::shutdown();
}

TEST_CASE("GetPeriod", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"timer_period_sec", "0.1"}};

    REQUIRE(node.GetPeriod(config_map) == 0.1);

    config_map.clear();
    REQUIRE_THROWS_AS(node.GetPeriod(config_map), std::runtime_error);

    rclcpp::shutdown();
}

TEST_CASE("GetMessageName", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"message_type", "TestMessage"}};

    REQUIRE(node.GetMessageName(config_map) == "TestMessage");

    config_map.clear();
    REQUIRE_THROWS_AS(node.GetMessageName(config_map), std::runtime_error);

    rclcpp::shutdown();
}

TEST_CASE("GetCallbackGroupName", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;
    std::map<std::string, std::string> config_map{{"callback_group", "TestGroup"}};

    REQUIRE(node.GetCallbackGroupName(config_map) == "TestGroup");

    config_map.clear();
    REQUIRE(node.GetCallbackGroupName(config_map) == "None"); // Default value

    rclcpp::shutdown();
}

TEST_CASE("ParseCallbackGroupConfig", "[ObeliskNode]") {
    rclcpp::init(0, nullptr);

    TestObeliskNode node;

    SECTION("Valid config") {
        node.ParseCallbackGroupConfig("group1:MutuallyExclusiveCallbackGroup,group2:ReentrantCallbackGroup");
        // We can't directly access callback_groups_, so we'll test indirectly through GetCallbackGroupName
        REQUIRE(node.GetCallbackGroupName({{"callback_group", "group1"}}) == "group1");
        REQUIRE(node.GetCallbackGroupName({{"callback_group", "group2"}}) == "group2");
    }

    SECTION("Invalid config") {
        node.ParseCallbackGroupConfig("group1:InvalidType");
        REQUIRE(node.GetCallbackGroupName({{"callback_group", "group1"}}) == "group1");
    }

    rclcpp::shutdown();
}
