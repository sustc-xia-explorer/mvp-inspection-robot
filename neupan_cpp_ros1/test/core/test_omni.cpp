#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "neupan/neupan_planner.hpp"

using namespace neupan;
namespace {
Mat matrix(const YAML::Node& n) {
  Mat a(n.size(), n[0].size());
  for (int i=0; i<a.rows(); ++i) for (int j=0; j<a.cols(); ++j) a(i,j)=n[i][j].as<double>();
  return a;
}
Vec vector(const YAML::Node& n) {
  Vec a(n.size());
  for (int i=0; i<a.size(); ++i) a(i)=n[i].as<double>();
  return a;
}
Robot omni() { return Robot::rectangle(Kinematics::Omni, 10, .1, Vec2(.2, M_PI), Vec2(.3, 3), .495, .36); }
}

TEST(Omni, OriginalPythonLinearizationAndQP) {
  const auto data=YAML::LoadFile(std::string(NEUPAN_TEST_DATA_DIR)+"/omni_python_reference.json");
  const auto r=Robot::rectangle(Kinematics::Omni, data["T"].as<int>(), data["dt"].as<double>(),
      vector(data["max_speed"]), vector(data["max_acce"]), .495, .36);
  NRMPParams p;
  p.max_num=data["max_num"].as<int>(); p.q_s=vector(data["q_s"]);
  p.p_u=data["p_u"].as<double>(); p.bk=data["bk"].as<double>();
  p.eta=data["eta"].as<double>(); p.ro_obs=data["ro_obs"].as<double>();
  p.d_min=data["d_min"].as<double>(); p.d_max=data["d_max"].as<double>();
  NRMP solver(r,p);  // Reuse solver: direction-dependent sparsity must stay valid.
  double worst_u=0, worst_objective=0;
  for (const auto& c: data["cases"]) {
    const Mat3X ns=matrix(c["nom_s"]), rs=matrix(c["ref_s"]);
    const Mat2X nu=matrix(c["nom_u"]);
    const Vec ru=vector(c["ref_us"]);
    std::vector<Mat> fa; std::vector<Vec> fb;
    for (int t=0;t<r.T;++t) {
      Mat33 a; Mat32 b; Vec3 offset; r.linearize(ns.col(t),nu.col(t),a,b,offset);
      EXPECT_LT((a-matrix(c["linear"][t]["A"])).norm(),1e-7);
      EXPECT_LT((b-matrix(c["linear"][t]["B"])).norm(),1e-7);
      EXPECT_LT((offset-vector(c["linear"][t]["C"])).norm(),1e-7);
      EXPECT_LT((r.predict(ns.col(t),nu.col(t))-ns.col(t+1)).norm(),1e-9);
      fa.push_back(matrix(c["fa"][t])); fb.push_back(vector(c["fb"][t]));
    }
    auto result=solver.solve(ns,nu,rs,ru,fa,fb);
    ASSERT_TRUE(result.success);
    const double du=(result.u-matrix(c["opt_u"])).cwiseAbs().maxCoeff();
    const double objective=std::abs(result.objective-c["objective"].as<double>());
    worst_u=std::max(worst_u,du); worst_objective=std::max(worst_objective,objective);
    EXPECT_LT(du,2e-4);
    EXPECT_LT((result.s-matrix(c["opt_s"])).cwiseAbs().maxCoeff(),2e-5);
    EXPECT_LT(objective,2e-5);
    EXPECT_LT((result.s.row(2).array()-ns(2,0)).abs().maxCoeff(),1e-8);
    Mat3X different_yaw=rs; different_yaw.row(2).array()+=100.;
    EXPECT_NEAR(solver.objective(result.s,result.u,result.d,ns,rs,ru,fa,fb),
      solver.objective(result.s,result.u,result.d,ns,different_yaw,ru,fa,fb),1e-10);
  }
  std::printf("Original Python omni: 5 QPs, max control error %.9g, max objective error %.9g\n",worst_u,worst_objective);
}

TEST(Omni, FrameConversionAndPolarFeedback) {
  const auto r=omni();
  const auto body=worldToBody(Vec2(.2,0), M_PI/2);
  EXPECT_NEAR(body.x(),0,1e-10); EXPECT_NEAR(body.y(),-.2,1e-10);
  EXPECT_LT((bodyToWorld(body,M_PI/2)-Vec2(.2,0)).norm(),1e-10);
  for (const Vec2 u: {Vec2(.1,3.13),Vec2(-.1,-2.8),Vec2(.1,-3.13)})
    EXPECT_LT((r.actionToControl(r.controlToAction(u),u)-u).norm(),1e-10);
  EXPECT_NEAR(r.actionToControl(Vec2::Zero(),Vec2(.1,1.5))(1),1.5,1e-10);
  // A speed reversal can preserve the crab angle instead of turning pi radians.
  const auto reversed=r.actionToControl(Vec2(-.1,0),Vec2(.1,0));
  EXPECT_NEAR(reversed(0),-.1,1e-10); EXPECT_NEAR(reversed(1),0,1e-10);
  EXPECT_THROW(parseKinematics("acker"),std::invalid_argument);
}

TEST(Omni, ConfigSwitchAndClosedLoop) {
  for (const auto& target: {Vec3(0,1,0),Vec3(1,1,M_PI/2),Vec3(-1,.7,-.8)}) {
    auto planner=NeuPANPlanner::fromYaml(std::string(NEUPAN_CONFIG_DIR)+"/fw_mini_omni.yaml",
                                       std::string(NEUPAN_MODEL_DIR)+"/fw_mini.bin");
    ASSERT_TRUE(planner.isOmni());
    Vec3 state(0,0,target(2));
    planner.updateInitialPathFromGoal(state,target);
    Vec2 previous=Vec2::Zero(); bool arrived=false;
    for (int i=0;i<500;++i) {
      NeuPANPlanner::Info info;
      const auto action=planner.forward(state,Mat2X(2,0),info);
      ASSERT_TRUE(action.allFinite());
      const auto control=planner.appliedControl();
      EXPECT_LE(std::abs(control(0)),.2+1e-9);
      EXPECT_LE(std::abs(control(0)-previous(0)),.03+1e-9);
      EXPECT_LE(std::abs(control(1)-previous(1)),.3+1e-9);
      previous=control;
      state.head<2>()+=.1*action;
      EXPECT_NEAR(state(2),target(2),1e-12);
      if(info.arrive) { arrived=true; break; }
    }
    EXPECT_TRUE(arrived) << "target=" << target.transpose() << " state=" << state.transpose();
    EXPECT_LT((state-target).head<2>().norm(),.11);
  }
  auto diff=NeuPANPlanner::fromYaml(std::string(NEUPAN_CONFIG_DIR)+"/fw_mini_balanced.yaml",
                                  std::string(NEUPAN_MODEL_DIR)+"/fw_mini.bin");
  EXPECT_FALSE(diff.isOmni());
}

TEST(Omni, EmergencyStopAndArrivalKeepTravelAngle) {
  auto planner=NeuPANPlanner::fromYaml(std::string(NEUPAN_CONFIG_DIR)+"/fw_mini_omni.yaml",
                                     std::string(NEUPAN_MODEL_DIR)+"/fw_mini.bin");
  Vec3 state(0,0,.7); planner.updateInitialPathFromGoal(state,Vec3(0,1,.7));
  for(int i=0;i<30;++i) { NeuPANPlanner::Info info; planner.forward(state,Mat2X(2,0),info); }
  const double angle=planner.appliedControl()(1);
  planner.setAppliedCommand(Vec2::Zero());
  EXPECT_NEAR(planner.appliedControl()(1),angle,1e-10);
  Mat2X collision(2,1); collision<<0,0;
  NeuPANPlanner::Info info;
  EXPECT_LT(planner.forward(state,collision,info).norm(),1e-10);
  EXPECT_TRUE(info.stop);
  EXPECT_NEAR(planner.appliedControl()(1),angle,1e-10);
  EXPECT_THROW(planner.setAppliedCommand(Vec2(2,0)),std::invalid_argument);
}

TEST(Omni, ArcLengthReferenceDoesNotDependOnWaypointDensity) {
  const auto r=omni();
  InitialPath::Options options;options.arc_length_sampling=true;options.close_threshold=0.;options.ind_range=40;
  Mat3X reference;
  for (const double spacing: {.005,.01,.04,1.}) {
    InitialPath path(r,.15,options);
    std::vector<InitialPath::PathPoint> points;
    const int count=static_cast<int>(std::ceil(1./spacing));
    for(int i=0;i<=count;++i)points.emplace_back(static_cast<double>(i)/count,0,0,1);
    path.setInitialPath(points);
    Vec3 state(0,0,0);EXPECT_FALSE(path.checkArrive(state));
    const auto nom=path.generateNomRefState(state,Mat2X::Zero(2,r.T),.15);
    if(reference.size()==0)reference=nom.ref_s;
    EXPECT_LT((reference-nom.ref_s).norm(),1e-10);
    EXPECT_NEAR(nom.ref_s(0,r.T),r.T*.15*r.dt,1e-10);
  }
}

TEST(Omni, ControlChangePenaltySuppressesDirectionChatter) {
  const auto r=omni();
  Mat3X ns=Mat3X::Zero(3,r.T+1),rs=ns;
  Mat2X nu=Mat2X::Zero(2,r.T);nu.row(0).setConstant(.15);
  for(int t=0;t<=r.T;++t) {
    ns(0,t)=t*.015;rs(0,t)=t*.015;
    rs(1,t)=t%2 ? .018 : -.018;
  }
  NRMPParams base;base.max_num=0;base.bk=0.;base.eta=0.;base.d_min=0.;base.d_max=0.;
  base.q_s=Vec3(4,4,1);
  const Vec2 applied(.15,0);
  NRMP legacy(r,base);const auto a=legacy.solve(ns,nu,rs,Vec::Constant(r.T,.15),{},{},&applied);
  ASSERT_TRUE(a.success);
  auto smooth=base;smooth.control_change_weight=Vec2(2,1);
  NRMP regularized(r,smooth);const auto b=regularized.solve(ns,nu,rs,Vec::Constant(r.T,.15),{},{},&applied);
  ASSERT_TRUE(b.success);
  auto variation=[](const Mat2X& u) {double value=std::abs(u(1,0));for(int t=1;t<u.cols();++t)value+=std::abs(u(1,t)-u(1,t-1));return value;};
  EXPECT_LT(variation(b.u),variation(a.u));
  EXPECT_NEAR(regularized.objective(b.s,b.u,b.d,ns,rs,Vec::Constant(r.T,.15),{},{},&applied),b.objective,1e-9);
}

TEST(Omni, ArcLengthReferenceUsesSignedSpeedForReverseTravel) {
  const auto r=omni();InitialPath::Options options;options.arc_length_sampling=true;
  InitialPath path(r,.15,options);
  path.setInitialPath({InitialPath::PathPoint(0,0,0,1),InitialPath::PathPoint(-1,0,0,1)});
  Mat2X u=Mat2X::Zero(2,r.T);u.row(0).setConstant(-.1);
  const auto reference=path.generateNomRefState(Vec3::Zero(),u,.15);
  EXPECT_LT(reference.ref_us.maxCoeff(),0.);
  EXPECT_LT(reference.ref_s(0,1),0.);
}

TEST(Omni, PeriodicDirectionPreservesXYAcrossThePiBoundary) {
  auto planner=NeuPANPlanner::fromYaml(std::string(NEUPAN_CONFIG_DIR)+"/fw_mini_omni.yaml",
                                     std::string(NEUPAN_MODEL_DIR)+"/fw_mini.bin");
  ASSERT_TRUE(planner.config().periodic_direction);
  Vec2 previous=Vec2::Zero();
  for(int k=0;k<110;++k) {
    const double angle=k*.06;
    const Vec2 velocity(.1*std::cos(angle),.1*std::sin(angle));
    planner.setAppliedCommand(velocity);
    const auto u=planner.appliedControl();
    EXPECT_LE(std::abs(u(1)),M_PI+1e-8);
    EXPECT_LT((planner.robot().controlToAction(u)-velocity).norm(),1e-9);
    EXPECT_LE(std::abs(wrapToPi(u(1)-previous(1))),.06+1e-8);
    previous=u;
  }
}

TEST(Omni, TurnEnvelopeBrakesBeforeTheCornerWithoutSlowingDistantStraight) {
  const auto r=omni();InitialPath::Options legacy;
  legacy.arc_length_sampling=true;legacy.close_threshold=0.;legacy.ind_range=100;
  auto anticipate=legacy;anticipate.turn_rate=.4;anticipate.turn_window=.16;
  std::vector<InitialPath::PathPoint> route={
      InitialPath::PathPoint(0,0,0,1),InitialPath::PathPoint(1,0,0,1),
      InitialPath::PathPoint(1,1,M_PI/2,1)};
  for(const double x: {.2,.85}) {
    InitialPath a(r,.15,legacy),b(r,.15,anticipate);a.setInitialPath(route);b.setInitialPath(route);
    const Vec3 state(x,0,0);a.checkArrive(state);b.checkArrive(state);
    Mat2X velocity=Mat2X::Zero(2,r.T);velocity.row(0).setConstant(.15);
    const auto unbounded=a.generateNomRefState(state,velocity,.15);
    const auto bounded=b.generateNomRefState(state,velocity,.15);
    if(x<.5)EXPECT_LT((bounded.ref_s-unbounded.ref_s).norm(),1e-10);
    else {
      EXPECT_LT(bounded.ref_us.minCoeff(),.10);
      EXPECT_GT(bounded.ref_us.minCoeff(),0.);
      EXPECT_LT(bounded.ref_s(0,r.T),unbounded.ref_s(0,r.T));
    }
  }
}
