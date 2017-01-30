#include <openhmd.h>
#include <string>
#include <vector>

class Rift {
  ohmd_context* ctx;
  ohmd_device* hmd;
  int num_devices;
  
 public:
  Rift();
  ~Rift();

  std::vector<float> rotation;
  void inputLoop();
  std::unordered_map<std::string, float> getDeviceInfo();
  std::vector<float> poll();
  void reset();

 private:
  void print(std::string name, int len, ohmd_float_value val);
  void sleep(double seconds);
  void connect();
};
