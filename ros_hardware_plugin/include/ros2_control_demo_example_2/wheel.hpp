#ifndef ESP32_WHEEL_HPP
#define ESP32_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name;      
  int enc;     
  double pos;   
  double vel; 
  double cmd;        
  double rad_per_count;  

  static constexpr int DEFAULT_TICKS_PER_REV = 255;

  Wheel()
      : name(""), enc(0), pos(0.0),
        vel(0.0), cmd(0.0),
        rad_per_count((2.0 * M_PI) / DEFAULT_TICKS_PER_REV) {}

  Wheel(const std::string &wheel_name, int ticks_per_rev = DEFAULT_TICKS_PER_REV)
  {
    setup(wheel_name, ticks_per_rev);
  }

  void setup(const std::string &wheel_name, int ticks_per_rev = DEFAULT_TICKS_PER_REV)
  {
    name = wheel_name;
    enc = 0;
    pos = 0.0;
    vel = 0.0;
    cmd = 0.0;
    rad_per_count = (2.0 * M_PI) / static_cast<double>(ticks_per_rev);
  }

  void update(int new_enc_count, double cmd_rev_s, double actual_rev_s)
  {
    enc = new_enc_count;
    cmd = cmd_rev_s;
    pos = enc * rad_per_count;
    vel = actual_rev_s * 2.0 * M_PI;
  }

  double get_position_revs() const
  {
    return pos / (2.0 * M_PI);
  }
};

#endif // ESP32_WHEEL_HPP
