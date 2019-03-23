#include <ros/ros.h>

#include <peripheralBase.hpp>

// Includes for periperal modules.
#include <powerBoard.h>

/**
 *
 */

class peripheralManager
{
private:
  // List of classes
  ros::NodeHandle nodehandle;

  std::vector<peripheralBase> peripherals;

  std::vector<std::tuple<ros::Publisher, int>> updates;
public:
  // Initializes own NodeHandle
  peripheralManager();
  ~peripheralManager();

  template<class T>
  int init_peripheral(peripheralCreateInfo &createinfo);


  int init_done();
  /*
   * The update loop that refreshes each peripheral.
   */
  int update();
};

peripheralManager::peripheralManager():
  nodehandle(ros::NodeHandle("~"))
{
}

peripheralManager::~peripheralManager()
{
}

template<class T>
int peripheralManager::init_peripheral(peripheralCreateInfo &createinfo = {std::string("null")});
{
  if(!createinfo.peripheral_name.compare(std::string("null")))
    T peripheral(nodehandle, createinfo);
  else
    T peripheral(nodehandle);

  this->peripherals.push_back(T)
  return 0;
}

int main(int argc, char ** argv)
{
  peripheralManager mgr;


  mgr.init_done();

  while(ros::ok())
  {
    ros::spinOnce();
    // Delay
  }
}
