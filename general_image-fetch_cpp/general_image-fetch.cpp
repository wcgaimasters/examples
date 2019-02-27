// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <cppcodec/base64_rfc4648.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <iostream>

class my_ai
  : public aiwc::ai_base
{
  static constexpr std::size_t BYTES_PER_PIXEL = 3;

public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
    // you don't have any information of the game here
  }

private:
  void init()
  {
    // initialize variables to store image
    // do not directly modify 'img_bgr' image updates are done in a way that
    // only the parts of the old frame that have been changed are overwritten by the new data
    img_bgr.clear();
    img_bgr.resize(info.resolution[X] * info.resolution[Y] * BYTES_PER_PIXEL);
    cv_img = cv::Mat(info.resolution[Y], info.resolution[X], CV_8UC3, &img_bgr[0]);

    cv::namedWindow("Frame", 1);
    cv::imshow("Frame", cv_img);
    cv::waitKey(1);
  }

  void update(const aiwc::frame& f)
  {
    auto pixel_ptr = [&](auto& img, std::size_t x, std::size_t y) {
      return &img[(info.resolution[X] * y + x) * BYTES_PER_PIXEL];
    };

    // fetch received subimages and construct a new frame
    // subimages are small regions on the new image
    // where some changes occurred since the last frame
    for(const auto& sub : f.subimages) {
      // the subframe data are encoded with base64 scheme
      const auto decoded = cppcodec::base64_rfc4648::decode(sub.base64);
      assert(decoded.size() == (sub.w * sub.h * BYTES_PER_PIXEL));

      auto* decoded_ptr = &decoded[0];

      // paste the fetched subframe into the frame data you already have
      // to update the image frame
      for(std::size_t y = 0; y < sub.h; ++y) {
        std::memcpy(pixel_ptr(img_bgr, sub.x, sub.y + y),
                    decoded_ptr,
                    sub.w * BYTES_PER_PIXEL);
        decoded_ptr += sub.w * BYTES_PER_PIXEL;
      }
    }

    // now img_bgr contains the updated frame
    cv::imshow("Frame", cv_img);
    cv::waitKey(1);

    if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
  std::vector<unsigned char> img_bgr;
  cv::Mat cv_img;
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
