#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "class_loader/class_loader.hpp"
#include <memory>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "crazyflie_sitl/crazyflie_sitl.hpp"
#include <tf2_ros/transform_broadcaster.h>


std::string pkg_share = ament_index_cpp::get_package_share_directory("crazyflie_sitl");
std::string default_yaml = pkg_share + "/config/crazyflies.yaml";

class CrazyflieSITLContainerException : public std::runtime_error
{
public:
  explicit CrazyflieSITLContainerException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};
struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
    std::atomic_bool thread_initialized;
    /// Constructor for the wrapper.
    /// This is necessary as atomic variables don't have copy/move operators
    /// implemented so this structure is not copyable/movable by default
    explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
    : executor(exec),
      thread_initialized(false)
    {
    }
  };
class CrazyflieSITContainer : public rclcpp::Node
{
public:
CrazyflieSITContainer(
    std::weak_ptr<rclcpp::Executor> executor,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crazyflie_stil_container", options)
  , m_executor(executor)
  , m_crazyflies()
  {
    this->declare_parameter<std::string>("crazyflie_configuration", default_yaml);
    std::string yaml_file;
    this->get_parameter("crazyflie_configuration", yaml_file);
    YAML::Node config = YAML::LoadFile(yaml_file);
    if (!config["crazyflies"])
    {
        RCLCPP_ERROR(this->get_logger(), "No 'crazyflies' key found in YAML file");
        return;
    }

    for (const auto &cf_ : config["crazyflies"])
    {
        int id = cf_["id"].as<int>();
        if (id < 0 || id > 255) {
            RCLCPP_ERROR(get_logger(), "Invalid ID '%d'. IDs must be between 0 and 255.", id);
            continue;
        }
        p_ids.push_back(id);
        p_initial_positions.push_back(cf_["initial_position"].as<std::vector<double>>());
    }

    if (config["publish_tf"])
    {
        if ((p_publish_tf = config["publish_tf"].as<bool>()) == true)
            RCLCPP_INFO(this->get_logger(), "TF publishing enabled.");
    }
    if (config["broadcast_pointcloud"])
    {
        if ((p_broadcast_pointcloud = config["broadcast_pointcloud"].as<bool>()) == true)
        {
            if (config["broadcast_pointcloud_rate_hz"] && config["broadcast_pointcloud_topic_name"])
            {
                p_broadcast_pointcloud_rate_hz = config["broadcast_pointcloud_rate_hz"].as<int>();
                p_broadcast_pointcloud_topic_name = config["broadcast_pointcloud_topic_name"].as<std::string>();
                
                m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    p_broadcast_pointcloud_topic_name, rclcpp::QoS(1));
                m_pointcloud_timer = this->create_wall_timer(
                    std::chrono::milliseconds(1000 / p_broadcast_pointcloud_rate_hz),
                    std::bind(&CrazyflieSITContainer::m_pointcloud_timer_callback, this)
                );
                RCLCPP_INFO(this->get_logger(), "Pointcloud broadcasting enabled.");
            } else RCLCPP_ERROR(this->get_logger(), "Pointcloud broadcasting enabled but 'broadcast_pointcloud_rate_hz' or 'broadcast_pointcloud_topic_name' is missing in the YAML file.");               
        }     
    }

    m_factory = create_component_factory("crazyflie_sitl", "CrazyflieSITL");
    m_initialize_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CrazyflieSITContainer::m_initialize_timer_callback, this));
    RCLCPP_INFO(get_logger(), "CrazyflieSITL Container started.");
  }
  ~CrazyflieSITContainer() override
  {
    while (!m_crazyflies.empty()) {
      remove_crazyflie(m_crazyflies.begin()->first);
    }
    RCLCPP_INFO(get_logger(), "Removed all crazyflies.");
  }
private: 
  void m_pointcloud_timer_callback()
  {
    if (m_pointcloud_pub && !m_crazyflies.empty())
    {
        // Create a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;

        for (auto & cf : m_crazyflies)
        {
            /**
             * This is some bit of magic here. However for large number of crazyflies
             * we don't want each crazyflie to publish its own position (as in lighthouse system)
             * enabling pointcloud mode therefore acts like a motion capture system such as vicon or optitrack
             */
            auto node = cf.second.first.get_node_instance();
            auto rclcpp_node_ptr = std::static_pointer_cast<rclcpp::Node>(node);
            auto crazyflie_ptr = std::dynamic_pointer_cast<CrazyflieSITL>(rclcpp_node_ptr);
            if (!crazyflie_ptr) {
                RCLCPP_WARN(this->get_logger(), "Failed to downcast to CrazyflieSITL for id %d", cf.first);
                continue;
            }
            pcl::PointXYZ point;
            std::vector<double> pos = crazyflie_ptr->get_position();
            point.x = pos[0];
            point.y = pos[1];
            point.z = pos[2];
            cloud.points.push_back(point);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;  // unorganized point cloud
        cloud.is_dense = true;

        // Convert PCL cloud to ROS message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = "world"; // change to your frame


        m_pointcloud_pub->publish(output);
    }
  }

  void m_initialize_timer_callback()
  {
    for (int i = 0; i < p_ids.size(); i++)
    {
        add_crazyflie(p_ids[i], p_initial_positions[i]);
    }
    RCLCPP_INFO(get_logger(), "Initialized %d crazyflies.", m_crazyflies.size());
    m_initialize_timer->cancel();

  }
  bool add_crazyflie(int id, const std::vector<double> & initial_position)
  {
    
    if (m_crazyflies.count(id))
    {
      RCLCPP_INFO(get_logger(), "Crazyflie with id '%d' already exists.", id);
      return false;
    }
    auto options = m_create_node_options(id, initial_position);
    try {
      auto node = m_factory->create_node_instance(options);
      auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      exec->add_node(node.get_node_base_interface());
      auto entry = m_crazyflies.emplace(std::make_pair(id, std::make_pair(node, exec)));
      DedicatedExecutorWrapper & wrapper = entry.first->second.second;
      wrapper.executor = exec;
      auto & thread_initialized = wrapper.thread_initialized;
      wrapper.thread = std::thread(
        [exec, &thread_initialized ]() {
          thread_initialized = true;
          try {
            exec->spin();
          } catch (...) {}
        }
      );
    } catch (const std::exception & ex) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw CrazyflieSITLContainerException("Failed to create node instance: " + std::string(ex.what()));
    } catch (...) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw CrazyflieSITLContainerException("Component constructor threw an exception");
    }
    
    return true;
  }
  bool remove_crazyflie(int id)
  {
    auto crazyflie = m_crazyflies.find(id);
    if (crazyflie == m_crazyflies.end()) {
      RCLCPP_INFO(get_logger(), "Crazyflie with id '%d' does not exist.", id);
      return false;
    }
    
    if (!crazyflie->second.second.thread_initialized)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(1)); // This only happens when add and removed are called near simultaniously. 
    }
    crazyflie->second.second.executor->cancel();
    crazyflie->second.second.thread.join();
    m_crazyflies.erase(crazyflie);
    return true;
  }
private: 
  rclcpp::NodeOptions
  m_create_node_options(int id, const std::vector<double> & initial_position)
  {
    std::vector<std::string> remap_rules;
    remap_rules.push_back("--ros-args");
    remap_rules.push_back("-r");

    remap_rules.push_back("__node:=crazyflie_sitl_" + std::to_string(id));
    auto add_parameter = [&](const std::string &name, const std::string &value) {
      remap_rules.push_back("-p");
      remap_rules.push_back(name + ":=" + value);
    };
    add_parameter("id", std::to_string(id));

    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < initial_position.size(); i++) {
        ss << std::fixed << std::setprecision(3) << initial_position[i]; // ensure double formatting
        if (i < initial_position.size() - 1) ss << ", ";
    }
    ss << "]"; 

    add_parameter("initial_position", ss.str());
    add_parameter("publish_tf", p_publish_tf ? "true" : "false");
    
    auto options = rclcpp::NodeOptions()
      .arguments(remap_rules);
      return options;
  }
  std::vector<std::pair<std::string, std::string>>
  get_component_resources(
    const std::string & package_name, const std::string & resource_index) const
  {
    std::string content;
    std::string base_path;
    if (
      !ament_index_cpp::get_resource(
        resource_index, package_name, content, &base_path))
    {
      throw CrazyflieSITLContainerException("Could not find requested resource in ament index");
    }
    std::vector<std::pair<std::string, std::string>> resources;
    std::vector<std::string> lines = rcpputils::split(content, '\n', true);
    for (const auto & line : lines) {
      std::vector<std::string> parts = rcpputils::split(line, ';');
      if (parts.size() != 2) {
        throw CrazyflieSITLContainerException("Invalid resource entry");
      }
      std::filesystem::path library_path = parts[1];
      if (!library_path.is_absolute()) {
        library_path = (base_path / library_path);
      }
      resources.push_back({parts[0], library_path.string()});
    }
    return resources;
  }
  std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const std::string & package_name, const std::string & class_name)
  {
    auto resources = get_component_resources(package_name, "rclcpp_components");
    
    std::string library_path = resources[0].second;
    std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";
    class_loader::ClassLoader * loader;
    RCLCPP_DEBUG(get_logger(), "Load Library: %s", library_path.c_str());
    try {
      m_loader = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      throw CrazyflieSITLContainerException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      throw CrazyflieSITLContainerException("Failed to load library: unknown error");
    }
  
    loader = m_loader.get();
    auto classes = m_loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (const auto & clazz : classes) {
        RCLCPP_DEBUG(get_logger(), "Found class: %s", clazz.c_str());
      if (clazz == class_name || clazz == fq_class_name) {
        RCLCPP_DEBUG(get_logger(), "Instantiate class: %s", clazz.c_str());
        return loader->createInstance<rclcpp_components::NodeFactory>(clazz);

      }
    }
    return {};
  }
  rclcpp::TimerBase::SharedPtr m_initialize_timer;
  std::vector<int> p_ids;
  std::vector<std::vector<double>> p_initial_positions;
  bool p_publish_tf = true;
  bool p_broadcast_pointcloud = false;
  int  p_broadcast_pointcloud_rate_hz = 100; // Hz
  std::string p_broadcast_pointcloud_topic_name = "pointCloud";
  std::unique_ptr<class_loader::ClassLoader> m_loader;
  std::shared_ptr<rclcpp_components::NodeFactory> m_factory;
  std::map<uint8_t, std::pair<rclcpp_components::NodeInstanceWrapper, DedicatedExecutorWrapper>> m_crazyflies;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointcloud_pub;
  std::shared_ptr<rclcpp::TimerBase> m_pointcloud_timer;
protected:
  std::weak_ptr<rclcpp::Executor> m_executor;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<CrazyflieSITContainer>(exec, options);
  exec->add_node(node);
  exec->spin();
  return 0;
}