// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/random/random_device.hpp>

#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
  }

private:
  void init()
  {
    // initialize a thread that will do the main job of data analysis, decision making, etc.
    behavior_thread = std::thread([&]() { frame_skip(); });
  }

  void update(const aiwc::frame& f)
  {
    std::unique_lock<std::mutex> lck(frames.m);
    // whenever a frame is received from the server, the frame is pushed into a queue
    frames.q.push_back(f);
    lck.unlock();
    frames.cv.notify_one();
  }

  void frame_skip()
  {
    // this function runs in the behavior thread to do operations
    for(;;) {
      std::unique_lock<std::mutex> lck(frames.m);
      // wait until some data appears in the queue
      frames.cv.wait(lck, [&]() { return !frames.q.empty(); });

      std::vector<aiwc::frame> local_queue;
      // take the data into a local queue and empty the shared queue
      local_queue.swap(frames.q);
      lck.unlock();

      // you can ignore all frames but the most recent one,
      // or keep only resetting frames,
      // or do whatever you want.

      // this example keeps only the most recent frame.
      choose_behavior_which_takes_really_long_time(local_queue.back());

      if(local_queue.back().reset_reason == aiwc::GAME_END) {
        break;
      }
    }
  }

  void choose_behavior_which_takes_really_long_time(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_END) {
      return;
    }

    // heavy operations
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // now send data to the server.
    std::cout << "Long operation ended." << std::endl;
  }

  void finish()
  {
    behavior_thread.join();

    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
  std::thread behavior_thread;

  struct {
    std::vector<aiwc::frame> q;
    std::mutex m;
    std::condition_variable cv;
  } frames;
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
