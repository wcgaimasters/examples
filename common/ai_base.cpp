// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include <boost/asio.hpp> // need to be the first header to avoid winsock error in windows

#include "ai_base.hpp"

#include <autobahn/autobahn.hpp>
#include <msgpack.hpp>

#include <boost/optional.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

// converters for types
namespace msgpack {
  MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
    namespace adaptor {

      template<>
      struct convert<aiwc::team_info>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::team_info& v) const
        {
          enum { NAME, RATING };

          // FIXME: inefficient
          const auto m = o.as<std::map<std::string, msgpack::object> >();

          decltype(m)::const_iterator its[] = {
            m.find("name"),
            m.find("rating"),
          };

          if(std::any_of(std::cbegin(its), std::cend(its), [&](const auto& it) { return it == std::cend(m); })) {
            throw msgpack::type_error();
          }

          v = aiwc::team_info{ its[NAME]->second.as<decltype(aiwc::team_info::name)>(),
                               its[RATING]->second.as<decltype(aiwc::team_info::rating)>() };
          return o;
        }
      };

      template<>
      struct convert<aiwc::robot_coordinate>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::robot_coordinate& v) const
        {
          const auto coord = o.as<std::tuple<double, double, double, bool, bool> >(); // x, y, th, active, touch
          v = aiwc::robot_coordinate{ std::get<0>(coord),
                                      std::get<1>(coord),
                                      std::get<2>(coord),
                                      std::get<3>(coord),
                                      std::get<4>(coord) };
          return o;
        }
      };

      template<>
      struct convert<aiwc::ball_coordinate>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::ball_coordinate& v) const
        {
          const auto coord = o.as<std::array<double, 2> >();
          v = aiwc::ball_coordinate{ coord[0], coord[1] };
          return o;
        }
      };

      template <>
      struct convert<aiwc::coordinates>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::coordinates& v) const
        {
          const auto coords = o.as<std::tuple<std::vector<aiwc::robot_coordinate>,
                                              std::vector<aiwc::robot_coordinate>,
                                              aiwc::ball_coordinate> >();
          v = aiwc::coordinates{ { std::get<0>(coords), std::get<1>(coords), },
                                 std::get<2>(coords) };
          return o;
        }
      };

      template<>
      struct convert<aiwc::game_info>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::game_info& v) const
        {
          enum {
            FIELD, GOAL, PENALTY_AREA, GOAL_AREA,
            BALL_RADIUS, BALL_MASS,
            ROBOT_SIZE, ROBOT_HEIGHT, AXLE_LENGTH, ROBOT_BODY_MASS,
            WHEEL_RADIUS, WHEEL_MASS,
            MAX_LINEAR_VELOCITY, MAX_TORQUE,
            RESOLUTION, NUMBER_OF_ROBOTS, CODEWORDS, GAME_TIME, TEAM_INFO,
          };

          const auto m = o.as<std::map<std::string, msgpack::object> >();

          decltype(m)::const_iterator its[] = {
            m.find("field"),
            m.find("goal"),
            m.find("penalty_area"),
            m.find("goal_area"),
            m.find("ball_radius"),
            m.find("ball_mass"),
            m.find("robot_size"),
            m.find("robot_height"),
            m.find("axle_length"),
            m.find("robot_body_mass"),
            m.find("wheel_radius"),
            m.find("wheel_mass"),
            m.find("max_linear_velocity"),
            m.find("max_torque"),
            m.find("resolution"),
            m.find("number_of_robots"),
            m.find("codewords"),
            m.find("game_time"),
            m.find("team_info"),
          };

          if(std::any_of(std::cbegin(its), std::cend(its), [&](const auto& it) { return it == std::cend(m); })) {
            throw msgpack::type_error();
          }

          v = aiwc::game_info{its[FIELD]->second.as<decltype(aiwc::game_info::field)>(),
                              its[GOAL]->second.as<decltype(aiwc::game_info::goal)>(),
                              its[PENALTY_AREA]->second.as<decltype(aiwc::game_info::penalty_area)>(),
                              its[GOAL_AREA]->second.as<decltype(aiwc::game_info::goal_area)>(),
                              its[BALL_RADIUS]->second.as<decltype(aiwc::game_info::ball_radius)>(),
                              its[BALL_MASS]->second.as<decltype(aiwc::game_info::ball_mass)>(),
                              its[ROBOT_SIZE]->second.as<decltype(aiwc::game_info::robot_size)>(),
                              its[ROBOT_HEIGHT]->second.as<decltype(aiwc::game_info::robot_height)>(),
                              its[AXLE_LENGTH]->second.as<decltype(aiwc::game_info::axle_length)>(),
                              its[ROBOT_BODY_MASS]->second.as<decltype(aiwc::game_info::robot_body_mass)>(),
                              its[WHEEL_RADIUS]->second.as<decltype(aiwc::game_info::wheel_radius)>(),
                              its[WHEEL_MASS]->second.as<decltype(aiwc::game_info::wheel_mass)>(),
                              its[MAX_LINEAR_VELOCITY]->second.as<decltype(aiwc::game_info::max_linear_velocity)>(),
                              its[MAX_TORQUE]->second.as<decltype(aiwc::game_info::max_torque)>(),
                              its[RESOLUTION]->second.as<decltype(aiwc::game_info::resolution)>(),
                              its[NUMBER_OF_ROBOTS]->second.as<decltype(aiwc::game_info::number_of_robots)>(),
                              its[CODEWORDS]->second.as<decltype(aiwc::game_info::codewords)>(),
                              its[GAME_TIME]->second.as<decltype(aiwc::game_info::game_time)>(),
                              its[TEAM_INFO]->second.as<decltype(aiwc::game_info::team_infos)>()};

          return o;
        }
      };

      template<>
      struct convert<aiwc::subimage>
      {
        msgpack::object const& operator()(msgpack::object const& o, aiwc::subimage& v) const
        {
          enum { X, Y, W, H, B64, };

          // FIXME: inefficient
          const auto m = o.as<std::map<std::string, msgpack::object> >();

          decltype(m)::const_iterator its[] = {
            m.find("x"),
            m.find("y"),
            m.find("w"),
            m.find("h"),
            m.find("base64"),
          };

          if(std::any_of(std::cbegin(its), std::cend(its), [&](const auto& it) { return it == std::cend(m); })) {
            throw msgpack::type_error();
          }

          v = aiwc::subimage{its[X]->second.as<std::size_t>(),
                             its[Y]->second.as<std::size_t>(),
                             its[W]->second.as<std::size_t>(),
                             its[H]->second.as<std::size_t>(),
                             its[B64]->second.as<std::string>()};

          return o;
        }
      };

    } // namespace adaptor
  } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack


namespace aiwc {

  struct ai_base::impl
  {
    void stop()
    {
      if(work) {
        work.reset();

        // stop io_thread
        io.post([&]() { io.stop(); });
        io_thread->join();

        io.reset();
        io_thread.reset();
        event_thread.reset();
      }
    }

    boost::asio::io_service io;
    std::unique_ptr<boost::asio::io_service::work> work;

    std::unique_ptr<std::thread> io_thread;
    std::unique_ptr<std::thread> event_thread;

    std::mutex events_mutex;
    std::condition_variable events_cv;
    std::deque<msgpack::object_handle> events;

    std::shared_ptr<autobahn::wamp_tcp_transport> transport;
    std::shared_ptr<autobahn::wamp_session> session;
  };

  ai_base::ai_base(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : server_ip(std::move(server_ip))
    , port(port)
    , realm(std::move(realm))
    , key(std::move(key))
    , datapath(std::move(datapath))
    , pimpl(std::make_unique<impl>())
  {
    // start io thread
    pimpl->work = std::make_unique<decltype(pimpl->work)::element_type>(pimpl->io); // prevent io_.run() from returning immediately
    pimpl->io_thread = std::make_unique<std::thread>([&]() {
        pimpl->io.run();
      });
  }

  ai_base::~ai_base()
  {
    try {
      if(pimpl->io_thread) {
        pimpl->stop();
      }
    }
    catch(const std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
    catch(...) {
      std::cerr << "Unknown exception" << std::endl;
    }
  }

  void ai_base::run()
  {
    // std::cout << "server_ip: " << server_ip << std::endl;
    // std::cout << "port: " << port << std::endl;
    // std::cout << "realm: " << realm << std::endl;
    // std::cout << "key: " << key << std::endl;

    // transport and session
    boost::asio::ip::tcp::endpoint tcp_endpoint(boost::asio::ip::address::from_string(server_ip), port);
    pimpl->transport = std::make_shared<autobahn::wamp_tcp_transport>(pimpl->io, tcp_endpoint);
    pimpl->session   = std::make_shared<autobahn::wamp_session>(pimpl->io);
    pimpl->transport->attach(std::static_pointer_cast<autobahn::wamp_transport_handler>(pimpl->session));

    // connect to server
    pimpl->transport->connect().get();

    // create session and join realm
    pimpl->session->start().get();
    pimpl->session->join(realm).get();

    // call info
    info = pimpl->session->call("aiwc.get_info", std::make_tuple(key)).get().argument<decltype(info)>(0);

    // subscribe
    auto sub = pimpl->session->subscribe(key, // the frame topic name is the key
                                         [&](const autobahn::wamp_event& ev) {
                                           // this function runs in the io thread. don't process event here and just hand it to the ai thread.
                                           std::unique_lock<decltype(pimpl->events_mutex)> lck(pimpl->events_mutex);
                                           pimpl->events.emplace_back(msgpack::clone(ev.argument<msgpack::object>(0)));
                                           pimpl->events_cv.notify_one();
                                         }).get();

    // call custom init: init() is called here since virtual function should NEVER be called in constructor
    init();

    // call ready
    pimpl->session->call("aiwc.ready", std::tuple<std::string>{key}).get();

    frame f;
    bool end_of_frame = false;
    bool game_ended = false;

    // process events
    while(game_ended == false) {
      // wait until event received
      std::unique_lock<decltype(pimpl->events_mutex)> lck(pimpl->events_mutex);
      pimpl->events_cv.wait(lck, [&]() { return !pimpl->events.empty(); });

      auto local_events = std::move(pimpl->events);
      lck.unlock();

      while(!local_events.empty()) {
        const msgpack::object_handle& h_obj = local_events.front();
        const msgpack::object& obj = h_obj.get();

        assert(obj.type == msgpack::type::MAP);

        for(std::size_t i = 0; i < obj.via.map.size; ++i) {
          const auto& kv = obj.via.map.ptr[i];
          const auto& key = kv.key.as<std::string>();
          const auto& value = kv.val;

          if(key == "time")                { f.time            = value.convert(); }
          else if(key == "score")          { f.score           = value.convert(); }
          else if(key == "reset_reason")   { f.reset_reason    = value.convert(); }
          else if(key == "game_state")     { f.game_state      = value.convert(); }
          else if(key == "ball_ownership") { f.ball_ownership  = value.convert(); }
          else if(key == "half_passed")    { f.half_passed     = value.convert(); }
          else if(key == "subimages") {
            std::vector<aiwc::subimage> subs = value.convert();
            std::copy(std::make_move_iterator(subs.begin()),
                      std::make_move_iterator(subs.end()),
                      std::back_inserter(f.subimages));
          }
          else if(key == "coordinates")  {
            f.opt_coordinates = value.as<decltype(f.opt_coordinates)::value_type>();
          }
          else if(key == "EOF")          { end_of_frame      = value.convert(); }
        }

        local_events.pop_front();

        if(end_of_frame) {
          update(f);

          if(f.reset_reason == aiwc::GAME_END) {
            game_ended = true;
          }

          f = {};
          end_of_frame = false;
        }
      }
    }

    finish();

    // unsubscribe
    pimpl->session->unsubscribe(sub).get();

    pimpl->session->leave().get();
    pimpl->session->stop().get();
    pimpl->transport->detach();

    pimpl->session.reset();
    pimpl->transport.reset();
  }

  void ai_base::set_wheel(const std::array<double, 10>& wheels)
  {
    pimpl->session->call("aiwc.set_speed", std::make_tuple(key, wheels)).get();
  }

  void ai_base::commentate(const std::string& comment)
  {
    pimpl->session->call("aiwc.commentate", std::make_tuple(key, comment)).get();
  }

  // element of report is a paragraph
  void ai_base::report(const std::vector<std::string>& rep)
  {
    pimpl->session->call("aiwc.report", std::make_tuple(key, rep)).get();
  }

} // namespace aiwc
