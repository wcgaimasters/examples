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
          // goalkeeper simply executes goalkeeper algorithm on its own
          goalkeeper(0);

          // defenders and forwards can pass ball to each other if necessary
          std::array<size_t, 4> list = {1, 2, 3, 4};
          passing_play(list);
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
    prev_sender = sender;
    prev_receiver = receiver;
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

    // record the robot closer to the ball between the two too
    if (def_min_dist < fwd_min_dist){
        closest_idx = def_idx;
    }
    else{
        closest_idx = fwd_idx;
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

  // check if sender/receiver pair should be reset
  bool check_reset_condition(void)
  {
	  // if the time is over, setting is reset
	  if ((end_count > 0) && (end_count - cur_count < 0))
		  return true;

  	// if there is no sender and receiver is not in shoot chance, setting is cleared
	  if (sender > 5 && receiver < 5 && !shoot_chance(receiver))
		  return true;

    return false;
  }

  // check if a sender can be selected
  bool check_sender_condition(void)
  {
    // if any of my robot is near the ball, a sender is available
    for(int i = 1; i < 5; i++) {
      double distance = dist(cur_posture[i][X], cur_ball[X], cur_posture[i][Y], cur_ball[Y]);
    	if ((distance < 0.5) && cur_posture[i][ACTIVE]) {
    		return true;
    	}
    }
    return false;
  }

  // check if a receiver should be selected
  bool check_receiver_condition(void)
  {
	  // if a sender exists and receiver does not exist yet, a receiver can be set
	  if ((sender < 5) && (receiver > 5))
		  return true;

    return false;
  }

  // select a sender
  size_t set_sender(std::array<size_t, 4> _player_list)
  {
    std::array<double, 4> distance_list;

    // leave non-candidates out
    for (int i = 0; i < 4; i++) {
      if (_player_list[i] > 5) {
        distance_list[i] = 999;
        continue;
      }

      std::array<double, 2> predict_ball = predict_ball_location(3);
      double ball_distance = dist(predict_ball[X], cur_posture[_player_list[i]][X], predict_ball[Y], cur_posture[_player_list[i]][Y]);
      distance_list[i] = ball_distance;
    }

    double min = 999;
    size_t index;
    for(int i = 0; i < 4; i++) {
      if (distance_list[i] < min) {
        min = distance_list[i];
        index = _player_list[i];
      }
    }

    // if the distance between ball and sender is predicted to be less than 1, choose the closest robot as the sender
    if (min < 1.0)
      return index;

    // otherwise, there is no sender
    return 99;
  }

  // select a receiver
  size_t set_receiver(std::array<size_t, 4> _player_list)
  {
    std::array<double, 4> receiver_op_dist_list;

	  // the robot farthest from opponents will be the receiver
    for(int r = 0; r < 4; r++) {
      std::array<double, 5> temp_receiver_op_dist_list;

	    // if a robot is not active, set distance to a high value
      if (_player_list[r] > 5) {
        temp_receiver_op_dist_list[r] = 999;
        continue;
      }

      // the sender is not a receiver candidate
      if(r + 1 == sender) {
      	receiver_op_dist_list[r] = 999;
      	continue;
      }

	    // get the distance between the robot and opponents
      for (int op = 1; op < 5; op++) { // four non-goalkeeper robots
        double op_distance = dist(cur_posture[r][X], cur_posture_op[op][X], cur_posture[r][Y], cur_posture[op][Y]);
        temp_receiver_op_dist_list[op - 1] = op_distance;
      }

      double min = 999;
      for(int i = 0; i < 5; i++) {
        if (temp_receiver_op_dist_list[i] < min)
          min = temp_receiver_op_dist_list[i];
      }
      receiver_op_dist_list[r] = min; // save the shortest distance between this robot and one of opponents
    }

    std::array<double, 4> receiver_ball_list;

    for(int r = 0; r < 4; r++) {
      // if the minimum distance between player and opponent's player is less than 0.5, this robot cannot be receiver
      if ((receiver_op_dist_list[r] < 0.5) || receiver_op_dist_list[r] == 999) {
        receiver_ball_list[r] = 999;
        continue;
      }
      size_t index = _player_list[r];
      double receiver_ball_distance = dist(cur_ball[X], cur_posture[index][X], cur_ball[Y], cur_posture[index][Y]);
      receiver_ball_list[r] = receiver_ball_distance;
    }

    double min = 999;
    size_t min_index = 99;
    for(int i = 0; i < 4; i++) {
      if (receiver_ball_list[i] < min) {
        min = receiver_ball_list[i];
        min_index = _player_list[i];
      }
    }

    return min_index;
  }

  // let robot with id 'id' execute an action directed by 'mode'
  void actions(size_t id, int mode, std::array<double, 2> target_pts = {-1,-1}, std::array<double, 4> params = {-1,-1,-1,-1}, bool refine = false)
  {
    std::array<double, 4> _params;
    std::array<double, 2> _target_pts;
    if (id > 5)
      return;

    // if the player state is set to 'stop', force the mode to be 'stop'
    if (player_state[id] == STOP)
      mode = STOP;

    // actions are set differently based on the modes
    switch(mode) {
    case NONE:
      {
        // reset the robot's status
        if (sender == id) {
          sender = 99;
          touch[0] = false;
          touch[1] = false;
          touch[2] = false;
          touch[3] = false;
          touch[4] = false;
        }
        if (receiver == id)
          receiver = 99;
        player_state[id] = NONE;
      }
      break;
    case FOLLOW:
      {
        // let the robot follow the ball
        if (target_pts[0] == -1)
          _target_pts = predict_ball_location(3);
        else
          _target_pts = target_pts;
        if (params[0] == -1){
          _params[0] = 1.0;
          _params[1] = 3.0;
          _params[2] = 0.6;
          _params[3] = false;
        }
        if (refine)
          _params = set_pos_parameters(id, _target_pts, params);
        set_target_position(id, _target_pts[X], _target_pts[Y], _params[0], _params[1], _params[2], _params[3]);
        player_state[id] = FOLLOW;
      }
      break;
    case DRIBBLE:
      {
        // let the robot follow the ball but at a faster speed
        if (target_pts[0] == -1)
          _target_pts = cur_ball;
        else
          _target_pts = target_pts;
        if (params[0] == -1){
          _params[0] = 1.4;
          _params[1] = 5.0;
          _params[2] = 0.8;
          _params[3] = false;
        }
        if (refine)
          _params = set_pos_parameters(id, _target_pts, params);
        set_target_position(id, _target_pts[X], _target_pts[Y], _params[0], _params[1], _params[2], _params[3]);
        player_state[id] = DRIBBLE;
      }
      break;
    case KICK:
      {
        // kick the ball
        if (target_pts[0] == -1)
          _target_pts = cur_ball;
        else
          _target_pts = target_pts;
        if (params[0] == -1){
          _params[0] = 1.4;
          _params[1] = 5.0;
          _params[2] = 0.8;
          _params[3] = true;
        }
        if ((end_count == 0) && !(touch[id]))
          end_count = cur_count + 10;
        player_state[id] = KICK;
        if (touch[id])
          player_state[id] = STOP;
        else
          touch[id] = cur_posture[id][TOUCH];
        if (player_state[id] == STOP){
          _params[0] = 0;
          _params[1] = 0;
          _params[2] = 0;
          _params[3] = false;
        }
        set_target_position(id, _target_pts[X], _target_pts[Y], _params[0], _params[1], _params[2], _params[3]);
      }
      break;
    case STOP:
      {
        // stop while counter is on
        if (params[0] == -1){
          _params[0] = 0;
          _params[1] = 0;
          _params[2] = false;
        }
        set_wheel_velocity(id, _params[0], _params[1], _params[2]);
        if (end_count == 0)
          end_count = cur_count + 5;
        player_state[id] = STOP;
        if (end_count - 1 == cur_count)
          player_state[id] = NONE;
      }
      break;
    case BACKWARD:
      {
        // retreat from the current position
        if (target_pts[0] == -1){
          _target_pts[0] = cur_posture[id][X] + 0.2;
          _target_pts[1] = cur_posture[id][Y];
        }
        else
          _target_pts = target_pts;
        if (params[0] == -1){
          _params[0] = 1.4;
          _params[1] = 5.0;
          _params[2] = 0.8;
          _params[3] = false;
        }
        if (refine)
          _params = set_pos_parameters(id, _target_pts, params);
        set_target_position(id, _target_pts[X], _target_pts[Y], _params[0], _params[1], _params[2], _params[3]);
        player_state[id] = BACKWARD;
      }
      break;
    case POSITION:
      {
        // go toward target position
        set_target_position(id, target_pts[X], target_pts[Y], _params[0], _params[1], _params[2], _params[3]);
      }
      break;
    default:
      break;
    }
  }

  void pass_ball(void)
  {
    if ((prev_sender == receiver && receiver < 5) || (prev_receiver == sender && sender < 5)) {
      double sender_ball_dist = dist(cur_posture[sender][X], cur_ball[X], cur_posture[sender][Y], cur_ball[Y]);
      if (sender_ball_dist < 1.5) {
        sender = prev_sender;
        receiver = prev_receiver;
      }
    }
    receive_ball();
    send_ball();
  }

  void send_ball(void)
  {
    if (sender > 5)
      return;

    std::array<double, 2> null2 = {-1, -1};
    std::array<double, 4> null4 = {-1, -1, -1, -1};

    double goal_dist = dist(info.field[X] / 2, cur_posture[sender][X], 0, cur_posture[sender][Y]);

    // if the sender has a shoot chance, it tries to shoot
    if (shoot_chance(sender)){
      if (goal_dist > 0.3 * info.field[X] / 2){
        actions(receiver, DRIBBLE, null2, null4, true);
        return;
      }
      else{
        actions(sender, KICK);
        return;
      }
    }

    // if the receiver exists, get the distance between the sender and the receiver
    double sender_receiver_dist = 999;
    if (receiver < 5)
      double sender_receiver_dist = dist(cur_posture[sender][X], cur_posture[receiver][X], cur_posture[sender][Y], cur_posture[receiver][Y]);

    // if the sender is close to the receiver, the sender kicks the ball
    if (sender_receiver_dist != 999)
      if ((sender_receiver_dist < 0.3) && !cur_posture[receiver][TOUCH])
        actions(sender, KICK);
        return;

    bool ift = is_facing_target(sender, cur_ball[X], cur_ball[Y]);
    double theta_diff = is_facing_target_direction(sender, cur_ball[X], cur_ball[Y]);
    // after the sender kicks, it stops
    if (!ift){
      if (theta_diff > PI * 3/4){
        actions(sender, NONE);
        return;
      }
      else{
        actions(sender, FOLLOW, null2, null4, true);
        return;
      }
    }

    // if the ball is in front of the sender and sender is moving backward
    if (cur_posture[sender][X] < -0.8 * info.field[X] / 2){
      if ((cur_posture[sender][X] - prev_posture[sender][X]) < 0)
        actions(sender, BACKWARD);
    }
    actions(sender, DRIBBLE, null2, null4, true);
    return;
  }

  void receive_ball(void)
  {
    // if receiver does not exist, do nothing
    if (receiver > 5)
      return;

    std::array<double, 2> null2 = {-1, -1};
    std::array<double, 4> null4 = {-1, -1, -1, -1};

    double goal_dist = dist(info.field[X] / 2, cur_posture[receiver][X], 0, cur_posture[receiver][Y]);
    // if sender is in shoot chance, receiver does nothing(reset)
    if (shoot_chance(sender)) {
      actions(receiver,NONE);
      return;
    }
    // if receiver is in shoot chance, kick or dribble forwards to the goal
    if (shoot_chance(receiver)) {
      if (goal_dist > 0.3 * info.field[X] / 2) {
        actions(receiver, DRIBBLE, null2, null4, true);
        return;
      }
      else {
        actions(receiver, KICK);
        return;
      }
    }
    // if sender exists
    if (sender < 5) {
      bool s2risFace = is_facing_target(sender, cur_posture[receiver][X], cur_posture[receiver][Y], 4);
      bool r2sisFace = is_facing_target(receiver, cur_posture[sender][X], cur_posture[sender][Y], 4);
      // if sender and receiver directs each other
      if (s2risFace && r2sisFace) {
        if ((cur_posture[receiver][TH] > 0) || (cur_posture[receiver][TH] < -3)) {
          actions(receiver, FOLLOW, {prev_posture[receiver][X], prev_posture[receiver][Y]-0.5 * info.field[Y]});
          return;
        }
        actions(receiver, FOLLOW, {prev_posture[receiver][X], prev_posture[receiver][Y] + 0.5 * info.field[Y]});
        return;
      }
    }
    std::array<double, 2> r_point = cur_ball;
    // if sender exists, get the expected point to meet receiver
    if (sender < 5)
      r_point = receive_position();
    double receiver_ball_dist = dist(cur_ball[X], cur_posture[receiver][X], cur_ball[Y], cur_posture[receiver][Y]);
    // if ball is close to receiver
    if (receiver_ball_dist > 0.3 * info.field[X] / 2) {
      actions(receiver, FOLLOW, {r_point[X], r_point[Y]}, null4, true);
      return;
    }
    bool r2bisFace = is_facing_target(receiver, cur_ball[X], cur_ball[Y], 4);
    if (!r2bisFace){
      actions(receiver, FOLLOW, null2, null4, true);
      return;
    }
    // if receiver is moving to our goal area
    if (cur_posture[receiver][X] < -0.8 * info.field[X] / 2) {
      if ((cur_posture[receiver][X] - prev_posture[receiver][X])< 0)
        actions(receiver, BACKWARD);
    }
    actions(receiver, DRIBBLE);
    return;
  }

  std::array<double, 4> set_pos_parameters(size_t id, std::array<double, 2> target_pts, std::array<double, 4> params, double mult = 1.2)
  {
    std::array<double, 4> _params;
    double prev_dist = dist(prev_posture[id][X], target_pts[X], prev_posture[id][Y], target_pts[Y]);
    double cur_dist = dist(cur_posture[id][X], target_pts[X], cur_posture[id][Y], target_pts[Y]);
    if (cur_dist > prev_dist - 0.02){
      _params[0] = params[0] * mult;
      _params[1] = params[1] * mult;
      _params[2] = params[2] * mult;
      _params[3] = params[3];
    }
    return _params;

  }

  bool is_facing_target(size_t index, double x, double y, double div = 4)
  {
    double dx = x - cur_posture[index][X];
    double dy = y - cur_posture[index][Y];
    double ds = std::sqrt(dx * dx + dy * dy);
    double desired_th;
    if (ds == 0)
      desired_th = 0;
    else
      desired_th = std::acos(dx / ds);

    double theta = cur_posture[index][X];
    if (desired_th < 0)
      desired_th += PI * 2;
    if (theta < 0)
      theta += PI * 2;
    double diff_theta = std::abs(desired_th - theta);
    if (diff_theta > PI)
      diff_theta = std::min(diff_theta, PI * 2 - diff_theta);
    if ((diff_theta < PI / div) || (diff_theta > PI * (1 - 1 / div)))
      return true;
    return false;
  }

  double is_facing_target_direction(size_t index, double x, double y, double div = 4)
  {
    double dx = x - cur_posture[index][X];
    double dy = y - cur_posture[index][Y];
    double ds = std::sqrt(dx * dx + dy * dy);
    double desired_th;
    if (ds == 0)
      desired_th = cur_posture[index][X];
    else
      desired_th = std::acos(dx / ds);

    double theta = cur_posture[index][X];
    if (desired_th < 0)
      desired_th += PI * 2;
    if (theta < 0)
      theta += PI * 2;
    double diff_theta = std::abs(desired_th - theta);
    if (diff_theta > PI)
      diff_theta = std::min(diff_theta, PI * 2 - diff_theta);
    return diff_theta;
  }

  std::array<double, 2> receive_position(void)
  {
    int step = 5;
    double ball_receiver_dist = dist(cur_ball[X], cur_posture[receiver][X], cur_ball[Y], cur_posture[receiver][Y]);
    double prev_ball_receiver_dist = dist(prev_ball[X], prev_posture[receiver][X], prev_ball[Y], prev_posture[receiver][Y]);
    double diff_dist = prev_ball_receiver_dist - ball_receiver_dist;
    if (diff_dist > 0)
      step = ball_receiver_dist / diff_dist;
    step = std::min(step, 15);
    std::array<double, 2> predict_pass_point = predict_ball_location(step);
    double ball_goal_dist = dist(cur_ball[X], info.field[X] / 2, cur_ball[Y], 0);
    double prev_ball_goal_dist = dist(prev_ball[X], info.field[X] / 2, prev_ball[Y], 0);
    if (ball_goal_dist > prev_ball_goal_dist)
      predict_pass_point[X] = predict_pass_point[X] - 0.15;
    return predict_pass_point;

  }

  void default_rulebased(std::array<size_t, 4> player_list)
  {
    for(int p = 0; p < 4; p++){
      // ff player is sender, receiver or out, do nothing
      if (player_list[p] > 5){
        continue;
      }
      // ff this robot is stuck at field sides, move forward the center
      if (pow(prev_posture[player_list[p]][X] - cur_posture[player_list[p]][X], 2) + pow(prev_posture[player_list[p]][Y] - cur_posture[player_list[p]][Y], 2) < 5e-6 && std::abs(cur_posture[player_list[p]][Y] - info.field[Y] / 2) < info.robot_size[player_list[p]] / 2 + 1e-2){
        if (cur_posture[player_list[p]][Y] > 0){
          set_target_position(p, 0, 0, 1.4, 3.5, 0.4, false);
          continue;
        }
      }
      // default rulebased
      if (player_list[p] == 0){
        goalkeeper(0);
        continue;
      }
      if (player_list[p] == 1){
        defender(1);
        continue;
      }
      if (player_list[p] == 2){
        defender(2);
        continue;
      }
      if (player_list[p] == 3){
        forward(3);
        continue;
      }
      if (player_list[p] == 4){
        forward(4);
        continue;
      }
    }
  }

  std::array<size_t, 4> find_active_player(std::array<size_t, 4> ids)
  {
    std::array<size_t, 4> _ids;
    for (int i = 0; i < 4; i++){
	    if (cur_posture[ids[i]][ACTIVE]) {
  			_ids[i] = ids[i];
  	  }
    	else {
    		_ids[i] = 99;
    	}
    }
    return _ids;
  }

  void passing_play(std::array<size_t, 4> player_list)
  {
    // select only alive player among forwards and defenders
    std::array<size_t, 4> _player_list = find_active_player(player_list);
    cur_count = floor(time * 20 + 0.5);

    if (end_count == cur_count)
      end_count = 0;

    if (check_reset_condition()){
      sender = 99;
      receiver = 99;
      sender_touch = false;
    }
    // check if sender candidate exists
    if (check_sender_condition()){
      sender = set_sender(_player_list);
    }
    // check if receiver candidate exists
    if (check_receiver_condition()){
      receiver = set_receiver(_player_list);
    }
    if (sender < 5 && receiver < 5){
      pass_ball();
      for (int i = 0; i < 4; i++){
        // if player is sender
        if (sender == i + 1){
          _player_list[i] = 99;
        }
        // if player is receiver
        if (receiver == i + 1){
          _player_list[i] = 99;
        }
      }
    }
    default_rulebased(_player_list);
    return;
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
  int cur_count = 0;
  int end_count = 0;
  size_t prev_sender;
  size_t prev_receiver;
  size_t sender;
  bool sender_touch;
  size_t receiver;
  size_t def_idx;
  size_t fwd_idx;
  size_t closest_idx;
  std::array<bool, 5> player_state;
  enum { NONE, FOLLOW, DRIBBLE, KICK, STOP, BACKWARD, POSITION};

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
