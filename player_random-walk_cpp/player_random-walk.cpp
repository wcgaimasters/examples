// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/random/random_device.hpp>

#include <fstream>
#include <iostream>
#include <random>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath, double dur)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
    , dur(dur)
  {
  }

private:
  void init()
  {
  }

  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_END) {
      return;
    }

    if(f.time >= last_changed + dur) {
      last_changed = f.time;
      std::mt19937 rng{boost::random_device{}()};
      std::uniform_real_distribution<double> dist(-1, 1);

      std::array<double, 10> wheels;
      for(int i = 0; i < 2*info.number_of_robots; i++)
        wheels[i] = info.max_linear_velocity[i/2]*dist(rng);
      set_wheel(wheels);
    }
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
  double dur;
  double last_changed = -9999.;
};

int main(int argc, char *argv[])
{
  if(argc < 6) {
    std::cerr << "Usage " << argv[0] << " server_ip port realm key datapath" << std::endl;
    return -1;
  }

  const auto& server_ip = std::string(argv[1]);
  const auto& port      = boost::lexical_cast<std::size_t>(argv[2]);
  const auto& realm     = std::string(argv[3]);
  const auto& key       = std::string(argv[4]);
  const auto& datapath  = std::string(argv[5]);

  my_ai ai(server_ip, port, realm, key, datapath, 1.); // change velocity every second

  ai.run();

  return 0;
}
