import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from planar_kinematics import PlanarPlant,rotate_vector


class PlanarKinematics(unittest.TestCase):
    def robot(self,model):
        return dict(kinematics=model,max_speed=[.2,math.pi] if model=='omni' else [.2,.4],
                    max_acce=[.3,3.] if model=='omni' else [.3,.6])

    def test_lateral_rotated_frame_and_polar_rates(self):
        plant=PlanarPlant(self.robot('omni')); state=[0.,0.,math.pi/2]
        # World +x is body -y when the robot points world +y.
        request=rotate_vector((.15,0),-state[2]);previous=(0.,0.)
        for _ in range(100):
            action=plant.apply(request,state[2],.02)
            for i,bound in enumerate((.3,3.)):
                self.assertLessEqual(abs(plant.control[i]-previous[i]),bound*.02+1e-10)
            previous=plant.control;state=plant.integrate(state,action,.02)
        self.assertGreater(state[0],.2);self.assertAlmostEqual(state[1],0.)
        self.assertAlmostEqual(state[2],math.pi/2);self.assertLess(action[1],-.1)

    def test_reverse_preserves_angle_and_emergency_zero(self):
        plant=PlanarPlant(self.robot('omni'))
        for _ in range(40):plant.apply((.15,0),0,.02)
        for _ in range(80):plant.apply((-.15,0),0,.02)
        self.assertLess(plant.control[0],-.1);self.assertAlmostEqual(plant.control[1],0.)
        self.assertEqual(plant.apply((.2,.1),0,.02,True),(0.,0.))
        self.assertEqual(plant.control[0],0.)

    def test_diff_has_no_lateral_motion(self):
        plant=PlanarPlant(self.robot('diff'))
        self.assertEqual(plant.integrate((0.,0.,0.),(.1,0.),1.),[.1,0.,0.])
        state=plant.integrate((0.,0.,0.),(.1,.4),1.)
        self.assertAlmostEqual(state[2],.4);self.assertGreater(state[1],0.)



class PeriodicDirection(unittest.TestCase):
    def test_cross_pi_changes_direction_continuously(self):
        from planar_kinematics import control_difference
        robot=dict(kinematics='omni',max_speed=[.18,2*math.pi],max_acce=[.2,.8],periodic_direction=True)
        plant=PlanarPlant(robot);plant.control=(.15,math.pi-.01)
        before=plant.control
        target=(.15*math.cos(math.pi+.01),.15*math.sin(math.pi+.01))
        actual=plant.apply(target,0.,.1)
        self.assertLess(math.dist(actual,target),1e-10)
        self.assertLess(plant.control[1],0.)
        self.assertAlmostEqual(control_difference(plant.control,before,robot)[1],.02)

if __name__=='__main__':unittest.main()
