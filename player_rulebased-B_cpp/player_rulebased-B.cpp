#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>


class my_ai
  : public aiwc::ai_base
{
  static constexpr double PI = 3.1415926535;

public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
    , robot_wheels{}
  {
    std::cout << "I am ready." << std::endl;
  }

private:
  // this function is called at the beginning of a game
  void init()
  {
    // from here, you have access to game-specific constant information such as field dimensions
    // check example 'general_check-variables_cpp' to see what information are available

    // you can initialize some customvariables here
  }

  // copy coordinates from frames to different variables just for convenience
  auto get_coord(const aiwc::frame& f)
  {
    //Get data in the frame
    decltype(cur_posture) cur;
    decltype(cur_posture_op) cur_op;
      std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };

    for(std::size_t id = 0; id < 5; ++id) {
      cur[id][X]  = (*f.opt_coordinates).robots[MYTEAM][id].x;
      cur[id][Y]  = (*f.opt_coordinates).robots[MYTEAM][id].y;
      cur[id][TH] = (*f.opt_coordinates).robots[MYTEAM][id].th;
      cur[id][ACTIVE] = (*f.opt_coordinates).robots[MYTEAM][id].active;
      cur[id][TOUCH] = (*f.opt_coordinates).robots[MYTEAM][id].touch;

      cur_op[id][X]  = (*f.opt_coordinates).robots[OPPONENT][id].x;
      cur_op[id][Y]  = (*f.opt_coordinates).robots[OPPONENT][id].y;
      cur_op[id][TH] = (*f.opt_coordinates).robots[OPPONENT][id].th;
      cur_op[id][ACTIVE] = (*f.opt_coordinates).robots[MYTEAM][id].active;
      cur_op[id][TOUCH] = (*f.opt_coordinates).robots[MYTEAM][id].touch;
    }

    return std::make_tuple(cur, cur_op, pos_ball);
  }

  // this function is called at each timestep. in 'f', current step's information are stored
  // check example 'general_check-variables_cpp' to see what information are available here
  // you should implement an AI soccer algorithm that sets robot wheel velocities for each timestep here
  void update(const aiwc::frame& f)
  {
	  frames.push_back(f);
    if(f.reset_reason == aiwc::GAME_START) {
      previous_frame = f;
      std::tie(cur_posture, cur_posture_op, cur_ball) = get_coord(f);
      return;
    }
    else if(f.reset_reason == aiwc::EPISODE_END) {
      // EPISODE_END is sent instead of GAME_END when 'repeat' option is set to 'true'
      // to mark the end of episode
      // you can reinitialize the parameters, count the number of episodes done, etc. here

      // this example does not do anything at episode end
    }
    else if(f.reset_reason == aiwc::HALFTIME) {
      // halftime is met - from next frame, received_frame.half_passed will be set to True
      // although the simulation switches sides,
      // coordinates and images given to your AI soccer algorithm will stay the same
      // that your team is red and located on left side whether it is 1st half or 2nd half

      // this example does not do anything at halftime
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      return;
    }

    // update current robot and ball postures
    std::tie(cur_posture, cur_posture_op, cur_ball) = get_coord(f);
    find_closest_robot();

    // array for holding wheel speeds
    std::array<double, 10> ws;

    // act differently based on the current game state
    switch(f.game_state) {
		//////////////////////////////////////////////////////////////////////
      case aiwc::STATE_DEFAULT:
        {
          // robots functions in STATE_DEFAULT
          goalkeeper(0);
          defender(1);
          defender(2);
          forward(3);
          forward(4);
        }
        break;
		//////////////////////////////////////////////////////////////////////
      case aiwc::STATE_KICKOFF:
        {
          // if the ball belongs to my team, initiate kickoff
          if (f.ball_ownership)
            set_target_position(4, 0, 0, 1.4, 3.0, 0.4, false);
        }
        break;
		//////////////////////////////////////////////////////////////////////
      case aiwc::STATE_GOALKICK:
        {
          // if the ball belongs to my team,
          // drive the goalkeeper to kick the ball
          if (f.ball_ownership)
            set_wheel_velocity(0, info.max_linear_velocity[0], info.max_linear_velocity[0], true);
        }
        break;
		//////////////////////////////////////////////////////////////////////
      case aiwc::STATE_CORNERKICK:
        {
          // just play as simple as possible
          goalkeeper(0);
          defender(1);
          defender(2);
          forward(3);
          forward(4);
        }
        break;
		//////////////////////////////////////////////////////////////////////
      case aiwc::STATE_PENALTYKICK:
        {
          // if the ball belongs to my team,
          // drive the forward to kick the ball
          if (f.ball_ownership)
            set_wheel_velocity(4, info.max_linear_velocity[4], info.max_linear_velocity[4], true);
        }
        break;
      default:
        break;
    }

    // hand over current frame as previous frame to next step
    prev_ball = cur_ball;
    prev_posture = cur_posture;
    prev_posture_op = cur_posture_op;
    previous_frame = f;

    // collect the robot wheel speeds
    for(std::size_t id = 0; id < 5; ++id) {
      ws[2*id    ] = robot_wheels[id][0]; // left
      ws[2*id + 1] = robot_wheels[id][1]; // right
    }

    // send wheel speed data to the simulator
    set_wheel(ws); // this function is defined at ai_base.cpp(examples/common)
  }

  // this function is called when the game is over
  // you can save some data here if necessary
  void finish()
  {
    // you have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");

    // write something
    ofs << "time, a.score, b.score, ";
    ofs << "ball.x, ball.y" << std::endl;

    // close the file
    ofs.close();
  }

  // All functions below are used for rulebased robot controls used in update()
  // convert degree to radian
  double d2r(double deg) {
    return deg * PI / 180;
  }

  // convert radian to degree
  double r2d(double rad) {
    return rad * 180 / PI;
  }

  // measure the distance between two coordinates (x1, y1) and (x2, y2)
  double dist(double x1, double x2, double y1, double y2)
  {
    const auto dx = x1 - x2;
    const auto dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
  }

  // convert radian in (-inf, inf) range to (-PI, PI) range
  double trim_radian(double rad)
  {
    double adj_rad = rad;
    while (adj_rad > PI)
      adj_rad -= 2 * PI;
    while (adj_rad < -PI)
      adj_rad += 2 * PI;
    return adj_rad;
  }

  // set the left and right wheel velocities of robot with id 'id'
  // 'max_velocity' scales the velocities up to the point where at least one of wheel is operating at max velocity
  void set_wheel_velocity(std::size_t id, double l, double r, bool max_velocity)
  {
    double multiplier = 1;

    // wheel velocities need to be scaled so that none of wheels exceed the maximum velocity available
    // otherwise, the velocity above the limit will be set to the max velocity by the simulation program
    // if that happens, the velocity ratio between left and right wheels will be changed that the robot may not execute
    // turning actions correctly.
    if (std::abs(l) > info.max_linear_velocity[id] || std::abs(r) > info.max_linear_velocity[id] || max_velocity) {
      if (std::abs(l) > std::abs(r)) {
        multiplier = info.max_linear_velocity[id] / std::abs(l);
      }
      else {
        multiplier = info.max_linear_velocity[id] / std::abs(r);
      }

      l *= multiplier;
      r *= multiplier;
    }

    robot_wheels[id] = {l, r};
  }

  // let the robot with id 'id' move to a target position (x, y)
  // the trajectory to reach the target position is determined by several different parameters
  void set_target_position(std::size_t id, double x, double y, double scale, double mult_lin, double mult_ang, bool max_velocity)
  {
    const double damping = 0.35;
    double ka = 0;
    int sign = 1;

    // calculate how far the target position is from the robot
    const double dx = x - cur_posture[id][X];
    const double dy = y - cur_posture[id][Y];
    const double d_e = std::sqrt(dx * dx + dy * dy);

    // calculate how much the direction is off
    const double desired_th = (dx == 0 && dy == 0) ? (PI / 2) : std::atan2(dy, dx);
    double d_th = desired_th - cur_posture[id][TH];
    while(d_th > PI) d_th -= 2 * PI;
    while(d_th < -PI) d_th += 2 * PI;

    // based on how far the target position is, set a parameter that
    // decides how much importance should be put into changing directions
    // farther the target is, less need to change directions fastly
    if(d_e > 1) {        ka = 17/90; }
    else if(d_e > 0.5) { ka = 19/90; }
    else if(d_e > 0.3) { ka = 21/90; }
    else if(d_e > 0.2) { ka = 23/90; }
    else               { ka = 25/90; }

    // if the target position is at rear of the robot, drive backward instead
    if(d_th > d2r(95)) {
      d_th -= PI;
      sign = -1;
    }
    else if(d_th < d2r(-95)) {
      d_th += PI;
      sign = -1;
    }

    // if the direction is off by more than 85 degrees,
    // make a turn first instead of start moving toward the target
    if(std::abs(d_th) > d2r(85)) {
      set_wheel_velocity(id, -mult_ang * d_th, mult_ang * d_th, false);
    }
    // otherwise
    else {
      // scale the angular velocity further down if the direction is off by less than 40 degrees
      if(d_e < 5.0 && std::abs(d_th) < d2r(40)) {
	      ka = 0.1;
      }
      ka *= 4;

      // set the wheel velocity
      // 'sign' determines the direction [forward, backward]
      // 'scale' scales the overall velocity at which the robot is driving
      // 'mult_lin' scales the linear velocity at which the robot is driving
      // larger distance 'd_e' scales the base linear velocity higher
      // 'damping' slows the linear velocity down
      // 'mult_ang' and 'ka' scales the angular velocity at which the robot is driving
      // larger angular difference 'd_th' scales the base angular velocity higher
      // if 'max_velocity' is true, the overall velocity is scaled to the point
      // where at least one wheel is operating at maximum velocity
      set_wheel_velocity(id,
	       sign * scale * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) - mult_ang * ka * d_th),
	       sign * scale * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) + mult_ang * ka * d_th), max_velocity);
    }
  }

  // predict where the ball will be located after 'steps' steps
  std::array<double, 2> predict_ball_location(int steps)
  {
    double dx = cur_ball[X] - prev_ball[X];
    double dy = cur_ball[Y] - prev_ball[Y];
    return {cur_ball[X] + steps * dx, cur_ball[Y] + steps * dy};
  }

  // check if a certain position is inside the penalty area of 'team'
  bool in_penalty_area(std::array<double, 2> obj, std::size_t team)
  {
    if (std::abs(obj[Y]) > info.penalty_area[Y] / 2)
      return false;

    if (team == MYTEAM)
      return (obj[X] < -info.field[X] / 2 + info.penalty_area[X]);
    else
      return (obj[X] > info.field[X] / 2 - info.penalty_area[X]);
  }

  // check if the robot with id 'id' has a chance to shoot
  bool shoot_chance(size_t id)
  {
    double dx = cur_ball[X] - cur_posture[id][X];
    double dy = cur_ball[Y] - cur_posture[id][Y];

    // if the ball is located further on left than the robot, it will be hard to shoot
  	if (dx < 0) {
  		return false;
  	}

    // if the robot->ball direction aligns with opponent's goal, the robot can shoot
    double y = (info.field[X] / 2 - cur_ball[X]) * dy / dx + cur_posture[id][Y];
  	if (abs(y) < info.goal[Y] / 2) {
  		return true;
  	}
  	else {
  		return false;
  	}
  }

  // find a defender and a forward closest to the ball
  void find_closest_robot(void)
  {
    // find the closest defender
    int min_idx = 0;
    double min_dist = 9999.99;
    double def_min_dist = 9999.99;

    for (int i = 1; i < 3; i++) {
      auto measured_dist = dist(cur_ball[X], cur_posture[i][X], cur_ball[Y], cur_posture[i][Y]);
      if (measured_dist < min_dist) {
        min_dist = measured_dist;
        min_idx = i;
        def_idx = min_idx;
        def_min_dist = min_dist;
      }
    }

    // find the closest forward
    min_idx = 0;
    min_dist = 9999.99;
    double fwd_min_dist = 9999.99;

    for (int i = 3; i < 5; i++) {
      auto measured_dist = dist(cur_ball[X], cur_posture[i][X], cur_ball[Y], cur_posture[i][Y]);
      if (measured_dist < min_dist) {
        min_dist = measured_dist;
        min_idx = i;
        fwd_idx = min_idx;
        fwd_min_dist = min_dist;
      }
    }
  }

  // turn to face 'desired_th' direction
  void angle(std::size_t id, double desired_th)
  {
    double mult_ang = 0.4;
    double sign = 1;

    double d_th = desired_th - cur_posture[id][TH];
    d_th = trim_radian(d_th);

    // the robot instead puts the direction rear if the angle difference is large
    if (d_th > d2r(95)) {
        d_th -= PI;
        sign = -1;
    }
    else if (d_th < d2r(-95)) {
        d_th += PI;
        sign = -1;
    }
    set_wheel_velocity(id, -mult_ang * d_th, mult_ang * d_th, false);
  }

  // returns the angle toward a specific position from current robot posture
  double direction_angle(std::size_t id, double x, double y)
  {
    double dx = x - cur_posture[id][X];
    double dy = y - cur_posture[id][Y];

    if ((dx == 0) && (dy == 0))
      return (PI / 2);
    else
      return std::atan2(dy, dx);
  }

  // let the robot face toward specific direction
  void face_specific_position(std::size_t id, double x, double y)
  {
    double dx = x - cur_posture[id][X];
    double dy = y - cur_posture[id][Y];

    double desired_th;
    if ((dx == 0) && (dy == 0))
      desired_th = PI / 2;
    else
      desired_th = std::atan2(dy, dx);

    angle(id, desired_th);
  }

  // check if the ball is coming toward the robot
  bool ball_coming_toward_robot(std::size_t id)
  {
    double x_dir = std::abs(cur_posture[id][X] - prev_ball[X]) > std::abs(cur_posture[id][X] - cur_ball[X]);
    double y_dir = std::abs(cur_posture[id][Y] - prev_ball[Y]) > std::abs(cur_posture[id][Y] - cur_ball[Y]);

    // ball is coming closer
    if ((x_dir) && (y_dir))
      return true;
    else
      return false;
  }

  // a basic goalkeeper rulebased algorithm
  void goalkeeper(std::size_t id)
  {
    // default desired position
    double x = (-info.field[X] / 2) + (info.robot_size[id] / 2) + 0.05;
    double y = std::max(std::min(cur_ball[Y], (info.goal[Y] / 2 - info.robot_size[id] / 2)), -info.goal[Y] / 2 + info.robot_size[id] / 2);
    std::array<double, 2> a = {cur_posture[id][X], cur_posture[id][Y]};

    // if the robot is inside the goal, try to get out
    if (cur_posture[id][X] < -info.field[X] / 2){
      if (cur_posture[id][Y] < 0)
        set_target_position(id, x, cur_posture[id][Y] + 0.2, 1.4, 5.0, 0.4, false);
      else
        set_target_position(id, x, cur_posture[id][Y] - 0.2, 1.4, 5.0, 0.4, false);
    }
    // if the goalkeeper is outside the penalty area
    else if (!in_penalty_area(a, MYTEAM)){
      // return to the desired position
      set_target_position(id, x, y, 1.4, 5.0, 0.4, true);
    }
    // if the goalkeeper is inside the penalty area
    else{
      // if the ball is inside the penalty area
      if (in_penalty_area(cur_ball, MYTEAM)){
        // if the ball is behind the goalkeeper
        if (cur_ball[X] < cur_posture[id][X]){
          // if the ball is not blocking the goalkeeper's path
          if (std::abs(cur_ball[Y] - cur_posture[id][Y]) > 2 * info.robot_size[id]){
            // try to get ahead of the ball
            set_target_position(id, cur_ball[X] - info.robot_size[id], cur_posture[id][Y], 1.4, 5.0, 0.4, false);
          }
          else{
            // just give up and try not to make a suicidal goal
            angle(id, PI / 2);
          }
        }
        // if the ball is ahead of the goalkeeper
        else{
          double desired_th = direction_angle(id, cur_ball[X], cur_ball[Y]);
          double rad_diff = trim_radian(desired_th - cur_posture[id][TH]);
          // if the robot direction is too away from the ball direction
          if (rad_diff > PI / 3){
            // give up kicking the ball and block the goalpost
            set_target_position(id, x, y, 1.4, 5.0, 0.4, false);
          }
          else{
            // try to kick the ball away from the goal
            set_target_position(id, cur_ball[X], cur_ball[Y], 1.4, 3.0, 0.8, true);
          }
        }
      }
      // if the ball is not in the penalty area
      else{
        // if the ball is within alert range and y position is not too different
        if ((cur_ball[X] < -info.field[X] / 2 + 1.5 * info.penalty_area[X])
        && (std::abs(cur_ball[Y]) < 1.5 * info.penalty_area[Y] / 2)
        && (std::abs(cur_ball[Y] - cur_posture[id][Y]) < 0.2))
          face_specific_position(id, cur_ball[X], cur_ball[Y]);
        // otherwise
        else
          set_target_position(id, x, y, 1.4, 5.0, 0.4, true);
      }
    }
  }

  // a basic defender rulebased algorithm
  void defender(std::size_t id)
  {
    // if the robot is inside the goal, try to get out
    if (cur_posture[id][X] < -info.field[X] / 2){
      if (cur_posture[id][Y] < 0)
        set_target_position(id, -0.7 * info.field[X] / 2, cur_posture[id][Y] + 0.2, 1.4, 3.5, 0.6, false);
      else
        set_target_position(id, -0.7 * info.field[X] / 2, cur_posture[id][Y] - 0.2, 1.4, 3.5, 0.6, false);
      return;
    }
    // the defender may try to shoot if condition meets
    if ((id == def_idx) && (shoot_chance(id)) && (cur_ball[X] < 0.3 * info.field[X] / 2)){
      set_target_position(id, cur_ball[X], cur_ball[Y], 1.4, 5.0, 0.4, true);
      return;
    }
    // if this defender is closer to the ball than the other defender
    if (id == def_idx){
      // ball is on our side
      if (cur_ball[X] < 0){
        // if the robot can push the ball toward opponent's side, do it
        if (cur_posture[id][X] < cur_ball[X] - info.ball_radius)
          set_target_position(id, cur_ball[X], cur_ball[Y], 1.4, 5.0, 0.4, true);
        else{
          // otherwise go behind the ball
          if (std::abs(cur_ball[Y] - cur_posture[id][Y]) > 0.3)
            set_target_position(id, std::max(cur_ball[X] - 0.5, -info.field[X] / 2 + info.robot_size[id] / 2), cur_ball[Y], 1.4, 3.5, 0.6, false);
          else
            set_target_position(id, std::max(cur_ball[X] - 0.5, -info.field[X] / 2 + info.robot_size[id] / 2), cur_posture[id][Y], 1.4, 3.5, 0.6, false);
        }
      }
      else
        set_target_position(id, -0.7 * info.field[X] / 2, cur_ball[Y], 1.4, 3.5, 0.4, false);
    }
    // if this defender is not closer to the ball than the other defender
    else{
      // ball is on our side
      if (cur_ball[X] < 0){
        // ball is on our left
        if (cur_ball[Y] > info.goal[Y] / 2 + 0.15)
          set_target_position(id, std::max(cur_ball[X] - 0.5, -info.field[X] / 2 + info.robot_size[id] / 2 + 0.1), info.goal[Y] / 2 + 0.15, 1.4, 3.5, 0.4, false);
        // ball is on our right
        else if (cur_ball[Y] < -info.goal[Y] / 2 - 0.15)
          set_target_position(id, std::max(cur_ball[X] - 0.5, -info.field[X] / 2 + info.robot_size[id] / 2 + 0.1), -info.goal[Y] / 2 - 0.15, 1.4, 3.5, 0.4, false);
        // ball is in center
        else
          set_target_position(id, std::max(cur_ball[X] - 0.5, -info.field[X] / 2 + info.robot_size[id] / 2 + 0.1), cur_ball[Y], 1.4, 3.5, 0.4, false);
      }
      else{
        // ball is on right side
        if (cur_ball[Y] < 0)
          set_target_position(id, -0.7 * info.field[X] / 2, std::min(cur_ball[Y] + 0.5, info.field[Y] / 2 - info.robot_size[id] / 2), 1.4, 3.5, 0.4, false);
        // ball is on left side
        else
          set_target_position(id, -0.7 * info.field[X] / 2, std::max(cur_ball[Y] - 0.5, -info.field[Y] / 2 + info.robot_size[id] / 2), 1.4, 3.5, 0.4, false);
      }
    }
  }

  // a basic forward rulebased algorithm
  void forward(std::size_t id)
  {
    // if the robot is blocking the ball's path toward opponent side
    if ((cur_ball[X] > -0.3 * info.field[X] / 2) && (cur_ball[X] < 0.3 * info.field[X] / 2) && (cur_posture[id][X] > cur_ball[X] + 0.1) && (std::abs(cur_posture[id][Y] - cur_ball[Y]) < 0.3)){
      if (cur_ball[Y] < 0)
        set_target_position(id, cur_posture[id][X] - 0.25, cur_ball[Y] + 0.75, 1.4, 3.0, 0.8, false);
      else
        set_target_position(id, cur_posture[id][X] - 0.25, cur_ball[Y] - 0.75, 1.4, 3.0, 0.8, false);
      return;
    }

    // if the robot can shoot from current position
    if ((id == fwd_idx) && (shoot_chance(id))){
      std::array<double, 2> pred_ball = predict_ball_location(2);
      set_target_position(id, pred_ball[X], pred_ball[Y], 1.4, 5.0, 0.4, true);
      return;
    }

    // if the ball is coming toward the robot, seek for shoot chance
    if ((id == fwd_idx) && (ball_coming_toward_robot(id))){
      double dx = cur_ball[X] - prev_ball[X];
      double dy = cur_ball[Y] - prev_ball[Y];
      double pred_x = (cur_posture[id][Y] - cur_ball[Y]) * dx / dy + cur_ball[X];
      double steps = (cur_posture[id][Y] - cur_ball[Y]) / dy;

      // if the ball will be located in front of the robot
      if (pred_x > cur_posture[id][X]){
        double pred_dist = pred_x - cur_posture[id][X];
        // if the predicted ball location is close enough
        if ((pred_dist > 0.1) && (pred_dist < 0.3) && (steps < 10)){
          // find the direction towards the opponent goal and look toward it
          double goal_angle = direction_angle(id, info.field[X] / 2, 0);
          angle(id, goal_angle);
          return;
        }
      }
    }

    // if this forward is closer to the ball than the other forward
    if (id == fwd_idx){
      if (cur_ball[X] > -0.3 * info.field[X] / 2){
        // if the robot can push the ball toward opponent's side, do it
        if (cur_posture[id][X] < cur_ball[X] - info.ball_radius)
          set_target_position(id, cur_ball[X], cur_ball[Y], 1.4, 5.0, 0.4, true);
        else{
          // otherwise go behind th ball
          if (std::abs(cur_ball[Y] - cur_posture[id][Y]) > 0.3)
            set_target_position(id, cur_ball[X] - 0.2, cur_ball[Y], 1.4, 3.5, 0.6, false);
          else
            set_target_position(id, cur_ball[X] - 0.2, cur_posture[id][Y], 1.4, 3.5, 0.6, false);
        }
      }
      else
        set_target_position(id, -0.1 * info.field[X] / 2, cur_ball[Y], 1.4, 3.5, 0.4, false);
    }
    // if this forward is closer to the ball than the other forward
    else{
      if (cur_ball[X] > -0.3 * info.field[X] / 2){
        // ball is on our right
        if (cur_ball[Y] < 0)
          set_target_position(id, cur_ball[X] - 0.25, info.goal[Y] / 2, 1.4, 3.5, 0.4, false);
        // ball is on our left
        else
          set_target_position(id, cur_ball[X] - 0.25, -info.goal[Y] / 2, 1.4, 3.5, 0.4, false);
      }
      else{
        // ball is on right side
        if (cur_ball[Y] < 0)
          set_target_position(id, -0.1 * info.field[X] / 2, std::min(-cur_ball[Y] - 0.5, info.field[Y] / 2 - info.robot_size[id] / 2), 1.4, 3.5, 0.4, false);
        // ball is on left side
        else
          set_target_position(id, -0.1 * info.field[X] / 2, std::max(-cur_ball[Y] + 0.5, -info.field[Y] / 2 + info.robot_size[id] / 2), 1.4, 3.5, 0.4, false);
      }
    }
  }

private: // member variable
  aiwc::frame previous_frame;

  std::array<std::array<double, 5>, 5> cur_posture;	//X,Y,THETA,ACTIVE,TOUCH
  std::array<std::array<double, 5>, 5> cur_posture_op;
  std::array<std::array<double, 5>, 5> prev_posture;
  std::array<std::array<double, 5>, 5> prev_posture_op;
  std::array<double, 2> prev_ball;
  std::array<double, 2> cur_ball;
  double time;

  std::array<std::array<double, 2>, 5> robot_wheels;

  std::array<bool, 5> touch;
  size_t def_idx;
  size_t fwd_idx;

  std::vector<aiwc::frame> frames;
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
