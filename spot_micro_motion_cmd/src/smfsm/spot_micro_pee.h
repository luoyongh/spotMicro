#pragma once

#include "spot_micro_state.h"
#include "command.h"
#include <iostream>


class SpotMicroPeeState : public SpotMicroState {
 public:
  SpotMicroPeeState(); // Constructor
  ~SpotMicroPeeState(); // Destructor
  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk::BodyState* body_state_cmd);

  virtual void init(const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd,
                    SpotMicroMotionCmd* smmc); 

  // Returns current state name as a string
  virtual std::string getCurrentStateName() {
    return "Pee";
  }

 private:
  smk::BodyState cmd_state_;

  // Three filters for angle commands
  XyzFilters angle_cmd_filters_;
};

