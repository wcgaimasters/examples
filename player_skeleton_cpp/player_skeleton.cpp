// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
    // you don't have any information of the game here
  }

private:
  // this function is called at the beginning of a game
  void init()
  {
    // from here, you have access to game-specific constant information such as field dimensions
    // check example 'general_check-variables_cpp' to see what information are available

    // you can initialize some customvariables here
  }

  // this function is called at each timestep. in 'f', current step's information are stored
  // check example 'general_check-variables_cpp' to see what information are available here
  // you should implement an AI soccer algorithm that sets robot wheel velocities for each timestep here
  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_START) {
      std::cout << "Game started : " << f.time << std::endl;
    }
    if(f.reset_reason == aiwc::SCORE_MYTEAM) {
      // yay! my team scored!
      std::cout << "Myteam scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::SCORE_OPPONENT) {
      // T_T what have you done
      std::cout << "Opponent scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

    if(f.opt_coordinates) { // if the optional coordinates are given,
      // const auto& myteam   = f.opt_coordinates->robots[MYTEAM];
      // const auto& opponent = f.opt_coordinates->robots[OPPONENT];
      // const auto& ball     = f.opt_coordinates->ball;
      //
      // const auto& myteam0_x      = (*f.opt_coordinates).robots[MYTEAM][0].x;
      // const auto& myteam0_y      = (*f.opt_coordinates).robots[MYTEAM][0].y;
      // const auto& myteam0_th     = (*f.opt_coordinates).robots[MYTEAM][0].th;
      // const auto& myteam0_active = (*f.opt_coordinates).robots[MYTEAM][0].active;
      // const auto& myteam0_touch  = (*f.opt_coordinates).robots[MYTEAM][0].touch;
      //
      // for (int i = 0; i < 5; i++) {
      //   if ((*f.opt_coordinates).robots[MYTEAM][i].touch) {
      //     std::cout << "My robot " << i << " touched the ball" << std::endl;
      //   }
      // }
    }
    else { // given no coordinates, you need to get coordinates from image
    }

    std::array<double, 10> wheels;
    for(int i = 0; i < 2*info.number_of_robots; i++)
      wheels[i] = info.max_linear_velocity[i/2];
    set_wheel(wheels); // every robot will go ahead with maximum velocity
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
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

  my_ai ai(server_ip, port, realm, key, datapath);

  ai.run();

  return 0;
}
