#include <ros/ros.h>

#include <vector>
#include <tuple>

typedef struct peripheralBaseCreateInfo {
  std::string peripheralName;
  // ...
} peripheralBaseCreateInfo;

class peripheralBase {
private:
  // List of publishers to update
  // and their update frequency
  std::vector<std:tuple<ros::Publisher, double>> publishers;

  // List of services that the
  // peripheral is designed to offer
  // NOTE this could probably be scrapped
  //   and replaced with a publisher?
  std::vector<ros::ServiceServer> services;

  // List of services which this class uses
  std::vector<ros::ServiceServer> clients;
public:
  peripheralBase(ros::NodeHandle &nh);
  peripheralBase(ros::NodeHandle &nh, peripheralBaseCreateInfo pbci);
  ~peripheralBase();

  /*
   * This function registers a publisher and appends
   * to the list of publishers.
   */
  void virtual registerPublisher();

  /*
   * This function registers a service server and appends
   * to the list of services.
   */
  void virtual registerService();

  /*
   * This function registers a service client and appends
   * to the list of services.
   */
  void virtual registerService();

  void virtual update();
private:
  // This ctor is useless.
  peripheralBase();
}
