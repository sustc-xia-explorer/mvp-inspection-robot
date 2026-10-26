// Link the real driver object with its main renamed to yhs_driver_main.
// A POSIX pipe receives CAN frames. This test never opens a CAN device.
#include "yhs_can_control.h"
#include <cassert>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <limits>

namespace yhs_tool {
class CommandWatchdogTest {
 public:
  static int run() {
    int channel[2];
    assert(pipe(channel) == 0);
    fcntl(channel[0], F_SETFL, O_NONBLOCK);
    CanControl driver;
    driver.dev_handler_ = channel[1];
    driver.current_control_mode_ = MODE_CTRL_CMD;
    const auto frame = [&]() {
      driver.timerCallBack(ros::WallTimerEvent());
      can_frame next{}, command{};
      bool found=false;
      while (read(channel[0], &next, sizeof(next)) == sizeof(next)) {
        if ((next.can_id & CAN_EFF_MASK) == 0x18C4D1D0) { command=next; found=true; }
      }
      assert(found);
      return command;
    };
    const auto velocities = [](const can_frame& f) {
      const auto* d=f.data;
      const short x=(d[0]>>4) | (d[1]<<4) | ((d[2]&15)<<12);
      const short w=(d[2]>>4) | (d[3]<<4) | ((d[4]&15)<<12);
      const short y=(d[4]>>4) | (d[5]<<4) | ((d[6]&15)<<12);
      return std::vector<short>{x,w,y};
    };
    assert(velocities(frame()) == std::vector<short>({0,0,0}));
    yhs_can_msgs::ctrl_cmd::Ptr cmd(new yhs_can_msgs::ctrl_cmd);
    cmd->ctrl_cmd_gear=2; cmd->ctrl_cmd_x_linear=.15f;
    cmd->ctrl_cmd_z_angular=10.f; cmd->ctrl_cmd_y_linear=0.f;
    driver.ctrl_cmdCallBack(cmd);
    const auto moving=velocities(frame());
    assert(std::abs(moving[0]-150)<=1 && moving[1]==1000 && moving[2]==0);
    // Freeze ROS time. The driver timeout still uses elapsed wall time.
    ros::Time::setNow(ros::Time(100));
    driver.last_ctrl_cmd_time_=ros::WallTime::now()-ros::WallDuration(.4);
    assert(velocities(frame()) == std::vector<short>({0,0,0}));
    driver.ctrl_cmdCallBack(cmd);
    assert(velocities(frame())[0]>0);
    cmd->ctrl_cmd_x_linear=std::numeric_limits<float>::quiet_NaN();
    driver.ctrl_cmdCallBack(cmd);
    assert(velocities(frame()) == std::vector<short>({0,0,0}));
    close(channel[0]);
    std::cout << "PASS: startup zero, CAN packing, wall-time timeout with frozen ROS time, invalid-command stop; pipe only\n";
    return 0;
  }
};
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"fw_driver_pipe_test",ros::init_options::AnonymousName);
  return yhs_tool::CommandWatchdogTest::run();
}
