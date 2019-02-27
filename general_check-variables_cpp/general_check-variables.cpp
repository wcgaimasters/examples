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
  void init()
  {
    // now you have information of the game

    // Print received constant variables to the console
    std::cout << "======================================================" << std::endl;
    std::cout << "Game Time: " << info.game_time << " seconds" << std::endl;
    std::cout << "# of robots: " << info.number_of_robots << " robots" << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "Field Dimensions: " << info.field[X] << " m long, " << info.field[Y] << " m wide" << std::endl;
    std::cout << "Goal Dimensions: " << info.goal[X] << " m deep, " << info.goal[Y] << " m wide" << std::endl;
    std::cout << "Penalty Area Dimensions: " << info.penalty_area[X] << " m long, " << info.penalty_area[Y] << " m wide" << std::endl;
    std::cout << "Goal Area Dimensions: " << info.goal_area[X] << " m long, " << info.goal_area[Y] << " m wide" << std::endl;
    std::cout << "Image Resolution: " << info.resolution[X] << " x " << info.resolution[Y] << std::endl;
    std::cout << "======================================================" << std::endl;
    std::cout << "Ball Radius: " << info.ball_radius << " m" << std::endl;
    std::cout << "Ball Mass: " << info.ball_mass << " kg" << std::endl;
    std::cout << "======================================================" << std::endl;
    for (int i = 0; i < info.number_of_robots; i++) {
      std::cout << "Robot " << i << ":" << std::endl;
      std::cout << "  size: " << info.robot_size[i] << " m x " << info.robot_size[i] << " m" << std::endl;
      std::cout << "  height: " << info.robot_height[i] << " m" << std::endl;
      std::cout << "  axle length: " << info.axle_length[i] << " m" << std::endl;
      std::cout << "  body mass: " << info.robot_body_mass[i] << " kg" << std::endl;
      std::cout << "  wheel radius: " << info.wheel_radius[i] << " m" << std::endl;
      std::cout << "  wheel mass: " << info.wheel_mass[i] << " kg" << std::endl;
      std::cout << "  max linear velocity: " << info.max_linear_velocity[i] << " m/s" << std::endl;
      std::cout << "  max torque: " << info.max_torque[i] << " N*m" << std::endl;
      std::cout << "  codeword: " << info.codewords[i] << std::endl;
      std::cout << "======================================================" << std::endl;
    }
  }

  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_START) {
      std::cout << "Game started : " << f.time << std::endl;
    }
    if(f.reset_reason == aiwc::SCORE_MYTEAM) {
      std::cout << "My team scored : " << f.time << std::endl;
      std::cout << "Current Score: [" << f.score[MYTEAM] << ", " << f.score[OPPONENT] << "]" << std::endl;
    }
    else if(f.reset_reason == aiwc::SCORE_OPPONENT) {
      std::cout << "Opponent scored : " << f.time << std::endl;
      std::cout << "Current Score: [" << f.score[MYTEAM] << ", " << f.score[OPPONENT] << "]" << std::endl;
    }
    else if(f.reset_reason == aiwc::HALFTIME) {
      std::cout << "Halftime" << std::endl;
    }
    else if(f.reset_reason == aiwc::EPISODE_END) {
      std::cout << "Episode ended" << std::endl;
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

    std::cout << "Halftime passed? " << f.half_passed << std::endl;

    if(f.game_state == aiwc::STATE_KICKOFF)
      std::cout << "Kickoff [My kickoff? " << f.ball_ownership << "]" << std::endl;
    else if(f.game_state == aiwc::STATE_GOALKICK)
      std::cout << "Goalkick [My goalkick? " << f.ball_ownership << "]" << std::endl;
    else if(f.game_state == aiwc::STATE_CORNERKICK)
      std::cout << "Cornerkick [My cornerkick? " << f.ball_ownership << "]" << std::endl;
    else if(f.game_state == aiwc::STATE_PENALTYKICK)
      std::cout << "Penaltykick [My penaltykick? " << f.ball_ownership << "]" << std::endl;

    if(f.opt_coordinates) {
      const auto& myteam   = f.opt_coordinates->robots[MYTEAM];
      const auto& opponent = f.opt_coordinates->robots[OPPONENT];
      const auto& ball     = f.opt_coordinates->ball;

      std::cout << "======================================================" << std::endl;
      std::cout << "Ball: (" << ball.x << ", " << ball.y << ")" << std::endl;
      std::cout << "======================================================" << std::endl;

      // Try replacing 'myteam' with 'opponent' to check opponent robots' state
      for (int i = 0; i < info.number_of_robots; i++) {
        std::cout << "Robot " << i << ":" << std::endl;
        std::cout << "  position: (" << myteam[i].x << ", " << myteam[i].y << ")" << std::endl;
        std::cout << "  orientation: " << myteam[i].th << std::endl;
        std::cout << "  activeness: " << myteam[i].active << std::endl;
        std::cout << "  touch: " << myteam[i].touch << std::endl;
        std::cout << "======================================================" << std::endl;
      }
    }
    else { // given no coordinates, you need to get coordinates from image
    }

    std::array<double, 10> wheels;
    for(int i = 0; i < 2*info.number_of_robots; i++)
      wheels[i] = 0;
    set_wheel(wheels);
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
