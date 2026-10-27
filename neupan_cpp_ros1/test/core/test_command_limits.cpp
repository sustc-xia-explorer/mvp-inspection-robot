#include <gtest/gtest.h>
#include "neupan/command_limiter.hpp"
#include "neupan/neupan_planner.hpp"

using namespace neupan;

TEST(CommandLimits, ReversalsAndBrakingRespectBothAxes) {
  Vec2 previous = Vec2::Zero();
  const Vec2 speed(.2, .4), acce(.3, .6);
  for (int i = 0; i < 160; ++i) {
    const Vec2 target = i < 40 ? Vec2(20, -20) : (i < 80 ? Vec2(-20, 20) : Vec2::Zero());
    const double dt = i % 2 ? .017 : .023;
    const Vec2 out = limitCommand(target, previous, speed, acce, dt);
    EXPECT_TRUE((out.cwiseAbs().array() <= speed.array()+1e-12).all());
    EXPECT_TRUE(((out-previous).cwiseAbs().array() <= acce.array()*dt+1e-12).all());
    previous = out;
  }
  EXPECT_EQ(previous.norm(), 0.);
  EXPECT_THROW(limitCommand(Vec2::Zero(), previous, speed, acce, 0), std::invalid_argument);
}

TEST(CommandLimits, QpFirstStepFixedAcrossPanAndMovingReplan) {
  NeuPANPlanner::Config cfg;
  cfg.dune_checkpoint = std::string(NEUPAN_MODEL_DIR)+"/diff_default.bin";
  cfg.max_speed = Vec2(.2, .4);
  cfg.max_acce = Vec2(.3, .6);
  cfg.ref_speed = .15;
  cfg.pan.iter_num = 5;
  NeuPANPlanner planner(cfg);
  auto set_path = [&]() {
    planner.setInitialPath({{0.,0.,0.,1.}, {5.,0.,0.,1.}});
  };
  set_path();
  Vec2 previous = Vec2::Zero();
  const Mat2X empty(2,0);
  for (int i=0; i<8; ++i) {
    if (i == 4) { set_path(); planner.reset(false); }
    NeuPANPlanner::Info info;
    const Vec2 out = planner.forward(Vec3::Zero(), empty, info);
    ASSERT_TRUE(info.solved);
    EXPECT_TRUE(((out-previous).cwiseAbs().array() <= cfg.max_acce.array()*.1+1e-12).all());
    // Solver tolerance may be ~1e-4; final guard above is strict.
    EXPECT_TRUE(((info.opt_u.col(0)-previous).cwiseAbs().array() <= cfg.max_acce.array()*.1+.002).all());
    previous = out;
  }
  planner.setInitialPath({{4.99,0.,0.,1.}, {5.,0.,0.,1.}});
  planner.reset(false);
  planner.setAppliedCommand(Vec2(.1, .2));
  NeuPANPlanner::Info info;
  Vec2 out = planner.forward(Vec3(5.,0.,0.), empty, info);
  EXPECT_TRUE(info.braking);
  EXPECT_FALSE(info.arrive);
  EXPECT_NEAR(out(0), .07, 1e-12);
  EXPECT_NEAR(out(1), .14, 1e-12);
  for (int i=0;i<5;++i) out=planner.forward(Vec3(5.,0.,0.), empty, info);
  EXPECT_TRUE(info.arrive);
  EXPECT_DOUBLE_EQ(out.norm(), 0.);
}
