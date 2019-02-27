// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_AI_BASE_HPP
#define H_AI_BASE_HPP
#pragma once

#include <boost/optional.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace aiwc {

  struct robot_coordinate
  {
    double x;
    double y;
    double th;
    bool active;
    bool touch;
  };

  struct ball_coordinate
  {
    double x;
    double y;
  };

  struct coordinates
  {
    std::array<std::vector<robot_coordinate>, 2> robots;
    ball_coordinate ball;
  };

  struct team_info
  {
    std::string name;
    double rating;
  };

  struct game_info
  {
    std::array<double, 2> field;        // [x, y]
    std::array<double, 2> goal;         // [x, y]
    std::array<double, 2> penalty_area; // [x, y]
    std::array<double, 2> goal_area;    // [x, y]

    double ball_radius;         // m
    double ball_mass;           // kg

    std::array<double, 5> robot_size;          // [m, m, m, m, m]
    std::array<double, 5> robot_height;        // [m, m, m, m, m]
    std::array<double, 5> axle_length;         // [m, m, m, m, m]
    std::array<double, 5> robot_body_mass;     // [kg, kg, kg, kg, kg]

    std::array<double, 5> wheel_radius;        // [m, m, m, m, m]
    std::array<double, 5> wheel_mass;          // [kg, kg, kg, kg, kg]

    std::array<double, 5> max_linear_velocity; // [m/s, m/s, m/s, m/s, m/s]
    std::array<double, 5> max_torque;          // [N*m, N*m, N*m, N*m, N*m]

    std::array<std::size_t, 2> resolution; // [x, y]
    std::size_t number_of_robots;
    std::vector<std::size_t> codewords;
    double game_time;

    std::array<team_info, 2> team_infos;
  };

  enum reset_reason {
    NONE           = 0,
    GAME_START     = 1,
    SCORE_MYTEAM   = 2,
    SCORE_OPPONENT = 3,
    GAME_END       = 4,
    DEADLOCK       = 5,
    GOALKICK       = 6,
    CORNERKICK       = 7,
    PENALTYKICK    = 8,
    HALFTIME       = 9,
    EPISODE_END    = 10,

    // aliases
    SCORE_ATEAM = SCORE_MYTEAM,
    SCORE_BTEAM = SCORE_OPPONENT,
  };

  enum game_state {
    STATE_DEFAULT = 0,
    STATE_KICKOFF = 1,
    STATE_GOALKICK = 2,
    STATE_CORNERKICK = 3,
    STATE_PENALTYKICK = 4,
  };

  struct subimage
  {
    std::size_t x;
    std::size_t y;
    std::size_t w;
    std::size_t h;

    std::string base64;
  };

  struct frame
  {
    double time;
    std::array<std::size_t, 2> score; // [my team, opponent] for player, [a team, b team] for commentator
    std::size_t reset_reason;
    std::size_t game_state;
    bool ball_ownership;
    bool half_passed;

    std::vector<subimage> subimages;

    boost::optional<coordinates> opt_coordinates;
  };

  class ai_base
  {
  protected:
    enum { MYTEAM, OPPONENT, ATEAM = MYTEAM, BTEAM = OPPONENT };
    enum { X, Y, TH, ACTIVE, TOUCH };

  public:
    ai_base(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath);
    ~ai_base();

    // noncopyable but movable
    ai_base(const ai_base&) = delete;
    ai_base& operator=(const ai_base&) = delete;
    ai_base(ai_base&&) = default;
    ai_base& operator=(ai_base&&) = default;

    void run();

  protected:
    void set_wheel(const std::array<double, 10>& wheels);
    void commentate(const std::string& comment);
    void report(const std::vector<std::string>& rep);

  private:
    virtual void init() = 0;
    virtual void update(const frame& f) = 0;
    virtual void finish() = 0;

  protected:
    const std::string server_ip;
    const std::size_t port;
    const std::string realm;
    const std::string key;
    const std::string datapath;
    game_info info;

  private:
    struct impl;
    std::unique_ptr<impl> pimpl;
  };

} // namespace aiwc

#endif // H_AI_BASE_HPP
