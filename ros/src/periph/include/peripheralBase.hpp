#include <ros/ros.h>

#include <vector>
#include <tuple>
#include <cstdint>

typedef struct peripheralCreateInfo {
  std::string peripheral_name;
  // ...
} peripheralBaseCreateInfo;

class peripheralBase {
private:
  ros::NodeHandle &nodehandle;

  // List of publishers to update
  // and their corresponding update frequency in hertz
  // - if frequency is set to 0, update will happen once at start
  //   and the value will be posted for all topics to use.
  //   (this is the default behaviour with the topic default)
  std::vector<std:tuple<ros::Publisher, uint32_t>> publishers;

  // List of services that the
  // peripheral is designed to offer
  // TODO look into if this can be scrapped
  std::vector<ros::ServiceServer> services;

  // List of services which this class uses
  std::vector<ros::ServiceServer> clients;
public:
peripheralBase(ros::NodeHandle &nh);
  peripheralBase(ros::NodeHandle &nh, peripheralBaseCreateInfo &pbci);
  ~peripheralBase();

  /*
   * This function registers a publisher and appends
   * to the list of publishers.
   */
  template<class T>
  void virtual registerPublisher(const std::string &topic,
                                 uint32_t queue_size,
                                 int update_rate);

  /*
   * This function registers a service server and appends
   * to the list of services.
   */
  template<class T>
  void virtual registerServiceServer(const std::string &service,
                                     bool(T::*srv_func)(MReq&, MRes&),
                                     T* obj);

  /*
   * This function registers a service client and appends
   * to the list of services.
   */
  template<class T>
  void virtual registerServiceClient(const std::string &service);

  std::vector<std:tuple<ros::Publisher, int>>& virtual get_updates();

private:
  // This ctor is useless.
  peripheralBase();
}
