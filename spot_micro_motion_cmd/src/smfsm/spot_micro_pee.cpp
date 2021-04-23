#include "spot_micro_stand.h"

#include "spot_micro_transition_stand.h"
#include "spot_micro_walk.h"
#include "spot_micro_pee.h"
#include "spot_micro_motion_cmd.h"
#include "rate_limited_first_order_filter.h"

SpotMicroPeeState::SpotMicroPeeState() {
  // Construcotr, doesn't need to do anything, for now...
  //std::cout << "SpotMicroPeeState Ctor" << std::endl;
}

SpotMicroPeeState::~SpotMicroPeeState() {
  //std::cout << "SpotMicroPeeState Dtor" << std::endl;
}

void SpotMicroPeeState::handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc, 
                                   smk::BodyState* body_state_cmd) {

  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Pee State" << std::endl;
  }

  

  if (cmd.getStandCmd() == true) {
    // Call parent class's change state method
    changeState(smmc, std::make_unique<SpotMicroTransitionStandState>());

  } else {
    // Get command values
    cmd_state_.euler_angs.phi   = cmd.getPhiCmd();
    cmd_state_.euler_angs.theta = cmd.getThetaCmd();
    cmd_state_.euler_angs.psi   = cmd.getPsiCmd();
  
    // Set command to filters 
    angle_cmd_filters_.x.setCommand(cmd_state_.euler_angs.phi);
    angle_cmd_filters_.y.setCommand(cmd_state_.euler_angs.theta);
    angle_cmd_filters_.z.setCommand(cmd_state_.euler_angs.psi);

    // Run Filters and get command values
    body_state_cmd->euler_angs.phi =
        angle_cmd_filters_.x.runTimestepAndGetOutput();
    body_state_cmd->euler_angs.theta =
        angle_cmd_filters_.y.runTimestepAndGetOutput();
    body_state_cmd->euler_angs.psi = 
        angle_cmd_filters_.z.runTimestepAndGetOutput();

    body_state_cmd->xyz_pos = cmd_state_.xyz_pos;

    body_state_cmd->leg_feet_pos = cmd_state_.leg_feet_pos;

    // Set and publish command
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();
  }
}



void SpotMicroPeeState::init(const smk::BodyState& body_state,
                               const SpotMicroNodeConfig& smnc,
                               const Command& cmd,
                               SpotMicroMotionCmd* smmc) {
  // Set default stance
  cmd_state_.leg_feet_pos = smmc->getPeeStance();

  // End body state position and angles
  cmd_state_.euler_angs.phi = 0.0f;
  cmd_state_.euler_angs.theta = 0.0f;
  cmd_state_.euler_angs.psi = 0.0f;

  cmd_state_.xyz_pos.x = 0.0f;
  cmd_state_.xyz_pos.y = smnc.default_stand_height;
  cmd_state_.xyz_pos.z = 0.0f;

  float dt = smnc.dt;
  float tau = smnc.transit_tau;
  float rate_limit = smnc.transit_angle_rl;

  typedef RateLmtdFirstOrderFilter rlof;

  angle_cmd_filters_.x = 
      rlof(dt, tau, cmd_state_.euler_angs.phi, rate_limit);
  angle_cmd_filters_.y = 
      rlof(dt, tau, cmd_state_.euler_angs.theta, rate_limit);
  angle_cmd_filters_.z = 
      rlof(dt, tau, cmd_state_.euler_angs.psi, rate_limit);

}

