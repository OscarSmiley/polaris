#include <ros/ros.h>

#include <vector>
#include <tuple>

typedef struct peripheralCreateInfo {
  std::string peripheral_name;
  // ...
} peripheralBaseCreateInfo;

class peripheralBase {
private:
  // List of publishers to update
  // and their corresponding update frequency in hertz
  std::vector<std:tuple<ros::Publisher, int>> publishers;

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
  template<class T>
  void virtual registerPublisher(const std::string &topic,
                                 uint32_t queue_size);

  /*
   * This function registers a service server and appends
   * to the list of services.
   */
  template<class T>
  void virtual registerServiceServer(const std::string &service,
                                     bool(T::srv_func)(MReq&, MRes&),
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
