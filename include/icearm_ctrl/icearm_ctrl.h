//
// Created by biomech on 21.02.24.
//

#ifndef ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_
#define ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_

#include "../icearm_struct.cc"

class IceArmCtrl {
 public:
  IceArmCtrl();
  ~IceArmCtrl();

  bool update_current_pos();


};

#endif //ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_
