#include "peripheralBase.hpp"

peripheralBase::peripheralBase(ros::NodeHandle &nh):
  nodehandle(nh)
{
  // Register Publishers
  // Ideally reserve n space in the publishers vector
  // then add your n publishers
  registerPublisher("default", 1, 0);

  // Register ServiceServers

  // Register ServiceClients
}

peripheralBase::peripheralBase(ros::NodeHandle &nh, peripheralBaseCreateInfo &pbci):
  peripheralBase(nh)
{
  // Set class variables from pbci
}


template<class T>
void virtual registerPublisher(const std::string &topic,
                               uint32_t queue_size,
                               uint32_t update_rate)
{
  // Ternary is so if there is no update rate, latching is enabled.
  ros::Publisher publisher = nodehandle.advertise(topic, queue_size, (update_rate == 0) ? true : false);
  std::tuple<ros::Publisher, uint32_t> tuple;
  tuple = std::make_tuple<ros::Publisher, int32_t>(publisher, update_rate);
  publishers.push_back(tuple);
}

template<class T>
void virtual registerServiceServer(const std::string &service,
                                   bool(T::*srv_func)(MReq&, MRes&),
                                   T* obj)
{
  ros::ServiceServer sserver = nodehandle.advertiseService(service, srv_func, obj);
  services.push_back(sserver);
}

template<class T>
void virtual registerServiceClient(const std::string &service)
{
  ros::ServiceClient sclient = nodehandle.serviceClient(service);
  clients.push_back(sclient);
}
