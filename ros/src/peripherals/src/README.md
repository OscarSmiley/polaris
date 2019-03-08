# Peripherals Package

The package works by defining a peripheral in it's own class, then linking that peripheral through to the peripheralManager.

## Peripheral Classes

These classes must be constructed according to the following specification.

```
struct packageNameCreateInfo
{
...
}

class packageName
{
private:
// ROS Publishers
// ROS Services
// ROS ServiceClients

// Parameters
   parameter_t parameter

public:
  packageName(ros::NodeHandle &nh, struct packageNameCreateInfo &createinfo)
  {
    // Load all the values from nh.getParam("packageName_<parameter>", )
    
    // Load all the publishers/services/serviceclients here
  }

  void update()
  {
    // This is where publishers will do their publishing.
  }
  
  // Suplementary functions.
}
```

In order to facilitate passing information into each of the classes
