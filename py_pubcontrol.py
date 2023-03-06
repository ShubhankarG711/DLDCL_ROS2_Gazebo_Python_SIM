### DL-DCL ROS2(Galactic)-Gazebo(ver.11) Simulation Master Code ###
### by Shubhankar Gupta, Suresh Sundaram ###
### AIRL, Dept. of Aerospace Engg., IISc. Bangalore, India ###
### For more details, please check the associated AAAI-23 conference paper titled,
### 'Moving-Landmark assisted Distributed Learning based Decentralized
### Cooperative Localization (DL-DCL) with Fault Tolerance.' ###

from cmath import sqrt
from ctypes.wintypes import LPWIN32_FIND_DATAA
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random
import numpy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('py_pubcontrol_node')
        self.publisher_0 = self.create_publisher(Twist, '/robot0/cmd_vel', 10)
        self.publisher_1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.publisher_2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.publisher_3 = self.create_publisher(Twist, '/robot3/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.i0 = 0.0
        self.timer_0 = self.create_timer(timer_period, self.publish_message0)
        self.subscriber_0 = self.create_subscription(Odometry, '/robot0/odom', self.subscribe_message0, 10)
        self.subscriber_1 = self.create_subscription(Odometry, '/robot1/odom', self.subscribe_message1, 10)
        self.subscriber_2 = self.create_subscription(Odometry, '/robot2/odom', self.subscribe_message2, 10)
        self.subscriber_3 = self.create_subscription(Odometry, '/robot3/odom', self.subscribe_message3, 10)

    def subscribe_message0(self, msg):
       global rbt0_x_act,rbt0_y_act,rbt0_psi_act
       rbt0_x_act = msg.pose.pose.position.x
       rbt0_y_act = msg.pose.pose.position.y
       rbt0_psi_act = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def subscribe_message1(self, msg):
       global rbt1_x_act,rbt1_y_act,rbt1_psi_act
       rbt1_x_act = msg.pose.pose.position.x
       rbt1_y_act = msg.pose.pose.position.y
       rbt1_psi_act = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def subscribe_message2(self, msg):
       global rbt2_x_act,rbt2_y_act,rbt2_psi_act
       rbt2_x_act = msg.pose.pose.position.x
       rbt2_y_act = msg.pose.pose.position.y
       rbt2_psi_act = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def subscribe_message3(self, msg):
       global rbt3_x_act,rbt3_y_act,rbt3_psi_act
       rbt3_x_act = msg.pose.pose.position.x
       rbt3_y_act = msg.pose.pose.position.y
       rbt3_psi_act = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def publish_message0(self):
        global rbt0_x_act,rbt0_y_act,rbt0_psi_act
        global rbt1_x_act,rbt1_y_act,rbt1_psi_act
        global rbt2_x_act,rbt2_y_act,rbt2_psi_act
        global rbt3_x_act,rbt3_y_act,rbt3_psi_act
        global timestep_cnt
        global L0_pos,L1_pos,L2_pos,L3_pos
        global L0_psi,L1_psi,L2_psi,L3_psi
        global vxrbt0, vxrbt1, vxrbt2, vxrbt3
        global wzrbt0, wzrbt1, wzrbt2, wzrbt3
        global estposrbt0, estposrbt1, estposrbt2, estposrbt3
        global estpsirbt0, estpsirbt1, estpsirbt2, estpsirbt3
        global deltaT

        rbt_pos = [[rbt0_x_act,rbt0_y_act],[rbt1_x_act,rbt1_y_act],[rbt2_x_act,rbt2_y_act],[rbt3_x_act,rbt3_y_act]]
        rbt_pose = [[rbt0_x_act,rbt0_y_act,rbt0_psi_act],[rbt1_x_act,rbt1_y_act,rbt1_psi_act],[rbt2_x_act,rbt2_y_act,rbt2_psi_act],[rbt3_x_act,rbt3_y_act,rbt3_psi_act]]

        if timestep_cnt == 0.0:
            estposrbt0 = [rbt0_x_act,rbt0_y_act]
            estposrbt1 = [rbt1_x_act,rbt1_y_act]
            estposrbt2 = [rbt2_x_act,rbt2_y_act]
            estposrbt3 = [rbt3_x_act,rbt3_y_act]
            estpsirbt0 = rbt0_psi_act
            estpsirbt1 = rbt1_psi_act
            estpsirbt2 = rbt2_psi_act
            estpsirbt3 = rbt3_psi_act

        G = [[1,1,0,1],[1,1,1,0],[0,1,1,1],[1,0,1,1]]
        Gn = rand_comm(G)

        if timestep_cnt >= 150.0:
            non_ideal_IMU = 1
        else:
            non_ideal_IMU = 0
        caseNo = 3

        if non_ideal_IMU == 1:
            rbt0_x_IMU = rbt0_x_act
            rbt0_y_IMU = rbt0_y_act
            rbt0_psi_IMU = rbt0_psi_act

            rbt1_x_IMU = rbt1_x_act + random.choice([-5,5])
            rbt1_y_IMU = rbt1_y_act + random.choice([-5,5])
            rbt1_psi_IMU = rbt1_psi_act + random.choice([-5,5])*3*(math.pi/180)

            rbt2_x_IMU = rbt2_x_act + random.choice([-5,5])
            rbt2_y_IMU = rbt2_y_act + random.choice([-5,5])
            rbt2_psi_IMU = rbt2_psi_act + random.choice([-5,5])*3*(math.pi/180)

            rbt3_x_IMU = rbt3_x_act
            rbt3_y_IMU = rbt3_y_act
            rbt3_psi_IMU = rbt3_psi_act
        else:
            rbt0_x_IMU = rbt0_x_act
            rbt0_y_IMU = rbt0_y_act
            rbt0_psi_IMU = rbt0_psi_act

            rbt1_x_IMU = rbt1_x_act
            rbt1_y_IMU = rbt1_y_act
            rbt1_psi_IMU = rbt1_psi_act

            rbt2_x_IMU = rbt2_x_act
            rbt2_y_IMU = rbt2_y_act
            rbt2_psi_IMU = rbt2_psi_act

            rbt3_x_IMU = rbt3_x_act
            rbt3_y_IMU = rbt3_y_act
            rbt3_psi_IMU = rbt3_psi_act

        rbt_pos_IMU = numpy.array([[rbt0_x_IMU,rbt0_y_IMU],[rbt1_x_IMU,rbt1_y_IMU],[rbt2_x_IMU,rbt2_y_IMU],[rbt3_x_IMU,rbt3_y_IMU]])
        rbt_pose_IMU = numpy.array([[rbt0_x_IMU,rbt0_y_IMU,rbt0_psi_IMU],[rbt1_x_IMU,rbt1_y_IMU,rbt1_psi_IMU],[rbt2_x_IMU,rbt2_y_IMU,rbt2_psi_IMU],[rbt3_x_IMU,rbt3_y_IMU,rbt3_psi_IMU]])
        rbt_psi_IMU = numpy.array([rbt0_psi_IMU,rbt1_psi_IMU,rbt2_psi_IMU,rbt3_psi_IMU])

        neigh_rbt0_pose = rbt_pose_IMU[neighID(0,Gn),:]
        neigh_rbt1_pose = rbt_pose_IMU[neighID(1,Gn),:]
        neigh_rbt2_pose = rbt_pose_IMU[neighID(2,Gn),:]
        neigh_rbt3_pose = rbt_pose_IMU[neighID(3,Gn),:]

        rbt0_pose_PRJ = prjphse(vxrbt0,wzrbt0,estposrbt0,estpsirbt0,deltaT)
        rbt1_pose_PRJ = prjphse(vxrbt1,wzrbt1,estposrbt1,estpsirbt1,deltaT)
        rbt2_pose_PRJ = prjphse(vxrbt2,wzrbt2,estposrbt2,estpsirbt2,deltaT)
        rbt3_pose_PRJ = prjphse(vxrbt3,wzrbt3,estposrbt3,estpsirbt3,deltaT)

        rbt_pose_PRJ = numpy.array([rbt0_pose_PRJ, rbt1_pose_PRJ, rbt2_pose_PRJ, rbt3_pose_PRJ])

        neigh_rbt0_pose_prj = rbt_pose_PRJ[neighID(0,Gn),:]
        neigh_rbt1_pose_prj = rbt_pose_PRJ[neighID(1,Gn),:]
        neigh_rbt2_pose_prj = rbt_pose_PRJ[neighID(2,Gn),:]
        neigh_rbt3_pose_prj = rbt_pose_PRJ[neighID(3,Gn),:]

        if timestep_cnt >= 150:
            is_faulty0 = 0
            is_faulty1 = 0
            is_faulty2 = 1
            is_faulty3 = 1
        else:
            is_faulty0 = 0
            is_faulty1 = 0
            is_faulty2 = 0
            is_faulty3 = 0

        rbt = 0
        #delposerbt0 = [[0]*3 for i in range(len(neighID(rbt,Gn)))]
        #delposerbt0 = [0]*range(len(neighID(rbt,Gn)))
        delposrbt0 = [0]*(len(neighID(rbt,Gn)))
        delpsirbt0 = [0]*(len(neighID(rbt,Gn)))
        delposetgtrbt0 = RPSS_trgt(rbt,rbt_pose,is_faulty0)
        delpostgtrbt0 = delposetgtrbt0[0:2]
        delpsitgtrbt0 = delposetgtrbt0[2]
        cnt = 0
        for neigh in neighID(rbt,Gn):
            delposerbt0 = RPSS(rbt,neigh,rbt_pose,is_faulty0)
            delposrbt0[cnt] = delposerbt0[0:2]
            delpsirbt0[cnt] = delposerbt0[2]
            cnt += 1

        rbt = 1
        #delposerbt1 = [[0]*3 for i in range(len(neighID(rbt,Gn)))]
        #delposerbt1 = [0]*range(len(neighID(rbt,Gn)))
        delposrbt1 = [0]*(len(neighID(rbt,Gn)))
        delpsirbt1 = [0]*(len(neighID(rbt,Gn)))
        delposetgtrbt1 = RPSS_trgt(rbt,rbt_pose,is_faulty1)
        delpostgtrbt1 = delposetgtrbt1[0:2]
        delpsitgtrbt1 = delposetgtrbt1[2]
        cnt = 0
        for neigh in neighID(rbt,Gn):
            delposerbt1 = RPSS(rbt,neigh,rbt_pose,is_faulty1)
            delposrbt1[cnt] = delposerbt1[0:2]
            delpsirbt1[cnt] = delposerbt1[2]
            cnt += 1

        rbt = 2
        #delposerbt2 = [[0]*3 for i in range(len(neighID(rbt,Gn)))]
        #delposerbt2 = [0]*range(len(neighID(rbt,Gn)))
        delposrbt2 = [0]*(len(neighID(rbt,Gn)))
        delpsirbt2 = [0]*(len(neighID(rbt,Gn)))
        delposetgtrbt2 = RPSS_trgt(rbt,rbt_pose,is_faulty2)
        delpostgtrbt2 = delposetgtrbt2[0:2]
        delpsitgtrbt2 = delposetgtrbt2[2]
        cnt = 0
        for neigh in neighID(rbt,Gn):
            delposerbt2 = RPSS(rbt,neigh,rbt_pose,is_faulty2)
            delposrbt2[cnt] = delposerbt2[0:2]
            delpsirbt2[cnt] = delposerbt2[2]
            cnt += 1

        rbt = 3
        #delposerbt3 = [[0]*3 for i in range(len(neighID(rbt,Gn)))]
        #delposerbt3 = [0]*range(len(neighID(rbt,Gn)))
        delposrbt3 = [0]*(len(neighID(rbt,Gn)))
        delpsirbt3 = [0]*(len(neighID(rbt,Gn)))
        delposetgtrbt3 = RPSS_trgt(rbt,rbt_pose,is_faulty3)
        delpostgtrbt3 = delposetgtrbt3[0:2]
        delpsitgtrbt3 = delposetgtrbt3[2]
        cnt = 0
        for neigh in neighID(rbt,Gn):
            delposerbt3 = RPSS(rbt,neigh,rbt_pose,is_faulty3)
            delposrbt3[cnt] = delposerbt3[0:2]
            delpsirbt3[cnt] = delposerbt3[2]
            cnt += 1

        delposrbt = [delposrbt0, delposrbt1, delposrbt2, delposrbt3] # corrected this
        delpostgt = [delpostgtrbt0, delpostgtrbt1, delpostgtrbt2, delpostgtrbt3]

        delpsirbt = [delpsirbt0, delpsirbt1, delpsirbt2, delpsirbt3] # corrected this
        delpsitgt = [delpsitgtrbt0, delpsitgtrbt1, delpsitgtrbt2, delpsitgtrbt3]

        neigh_delposrbt0 = [delposrbt[i] for i in neighID(0,Gn)]
        neigh_delposrbt1 = [delposrbt[i] for i in neighID(1,Gn)]
        neigh_delposrbt2 = [delposrbt[i] for i in neighID(2,Gn)]
        neigh_delposrbt3 = [delposrbt[i] for i in neighID(3,Gn)]

        neigh_delpsirbt0 = [delpsirbt[i] for i in neighID(0,Gn)]
        neigh_delpsirbt1 = [delpsirbt[i] for i in neighID(1,Gn)]
        neigh_delpsirbt2 = [delpsirbt[i] for i in neighID(2,Gn)]
        neigh_delpsirbt3 = [delpsirbt[i] for i in neighID(3,Gn)]

        neigh_delpostgt0 = [delpostgt[i] for i in neighID(0,Gn)]
        neigh_delpostgt1 = [delpostgt[i] for i in neighID(1,Gn)]
        neigh_delpostgt2 = [delpostgt[i] for i in neighID(2,Gn)]
        neigh_delpostgt3 = [delpostgt[i] for i in neighID(3,Gn)]

        neigh_delpsitgt0 = [delpsitgt[i] for i in neighID(0,Gn)]
        neigh_delpsitgt1 = [delpsitgt[i] for i in neighID(1,Gn)]
        neigh_delpsitgt2 = [delpsitgt[i] for i in neighID(2,Gn)]
        neigh_delpsitgt3 = [delpsitgt[i] for i in neighID(3,Gn)]

        if caseNo == 1: ############################### IMU only
            rbt0_x_est = rbt0_x_IMU
            rbt0_y_est = rbt0_y_IMU
            rbt0_psi_est = rbt0_psi_IMU

            rbt1_x_est = rbt1_x_IMU
            rbt1_y_est = rbt1_y_IMU
            rbt1_psi_est = rbt1_psi_IMU

            rbt2_x_est = rbt2_x_IMU
            rbt2_y_est = rbt2_y_IMU
            rbt2_psi_est = rbt2_psi_IMU

            rbt3_x_est = rbt3_x_IMU
            rbt3_y_est = rbt3_y_IMU
            rbt3_psi_est = rbt3_psi_IMU

            estposrbt0 = [rbt0_x_est,rbt0_y_est]
            estposrbt1 = [rbt1_x_est,rbt1_y_est]
            estposrbt2 = [rbt2_x_est,rbt2_y_est]
            estposrbt3 = [rbt3_x_est,rbt3_y_est]

        elif caseNo == 2: ################################ RPSS only
            rbt0_x_est, rbt0_y_est, rbt0_psi_est =  RPSS_tgt(0,rbt_pose,is_faulty0)
            rbt1_x_est, rbt1_y_est, rbt1_psi_est =  RPSS_tgt(1,rbt_pose,is_faulty1)
            rbt2_x_est, rbt2_y_est, rbt2_psi_est =  RPSS_tgt(2,rbt_pose,is_faulty2)
            rbt3_x_est, rbt3_y_est, rbt3_psi_est =  RPSS_tgt(3,rbt_pose,is_faulty3)

            estposrbt0 = [rbt0_x_est,rbt0_y_est]
            estposrbt1 = [rbt1_x_est,rbt1_y_est]
            estposrbt2 = [rbt2_x_est,rbt2_y_est]
            estposrbt3 = [rbt3_x_est,rbt3_y_est]

        elif caseNo == 3: ################################ DLDCL1hop
            rbt0_x_est, rbt0_y_est, L0_pos = DLDCL1hop_pos(timestep_cnt,0,Gn,L0_pos,neigh_rbt0_pose,neigh_rbt0_pose_prj,neigh_delposrbt0,neigh_delpostgt0)
            estposrbt0 = [rbt0_x_est,rbt0_y_est]
            rbt0_psi_est, L0_psi = DLDCL1hop_psi(timestep_cnt,0,Gn,L0_psi,neigh_rbt0_pose,neigh_rbt0_pose_prj,neigh_delpsirbt0,neigh_delpsitgt0)
            estpsirbt0 = rbt0_psi_est

            rbt1_x_est, rbt1_y_est, L1_pos = DLDCL1hop_pos(timestep_cnt,1,Gn,L1_pos,neigh_rbt1_pose,neigh_rbt1_pose_prj,neigh_delposrbt1,neigh_delpostgt1)
            estposrbt1 = [rbt1_x_est,rbt1_y_est]
            rbt1_psi_est, L1_psi = DLDCL1hop_psi(timestep_cnt,1,Gn,L1_psi,neigh_rbt1_pose,neigh_rbt1_pose_prj,neigh_delpsirbt1,neigh_delpsitgt1)
            estpsirbt1 = rbt1_psi_est

            rbt2_x_est, rbt2_y_est, L2_pos = DLDCL1hop_pos(timestep_cnt,2,Gn,L2_pos,neigh_rbt2_pose,neigh_rbt2_pose_prj,neigh_delposrbt2,neigh_delpostgt2)
            estposrbt2 = [rbt2_x_est,rbt2_y_est]
            rbt2_psi_est, L2_psi = DLDCL1hop_psi(timestep_cnt,2,Gn,L2_psi,neigh_rbt2_pose,neigh_rbt2_pose_prj,neigh_delpsirbt2,neigh_delpsitgt2)
            estpsirbt2 = rbt2_psi_est

            rbt3_x_est, rbt3_y_est, L3_pos = DLDCL1hop_pos(timestep_cnt,3,Gn,L3_pos,neigh_rbt3_pose,neigh_rbt3_pose_prj,neigh_delposrbt3,neigh_delpostgt3)
            estposrbt3 = [rbt3_x_est,rbt3_y_est]
            rbt3_psi_est, L3_psi = DLDCL1hop_psi(timestep_cnt,3,Gn,L3_psi,neigh_rbt3_pose,neigh_rbt3_pose_prj,neigh_delpsirbt3,neigh_delpsitgt3)
            estpsirbt3 = rbt3_psi_est


        rbt0_phi = math.atan2(rbt0_y_est,rbt0_x_est)
        rbt1_phi = math.atan2(rbt1_y_est,rbt1_x_est)
        rbt2_phi = math.atan2(rbt2_y_est,rbt2_x_est)
        rbt3_phi = math.atan2(rbt3_y_est,rbt3_x_est)

        rbt0_r = distnce(0,0,rbt0_x_est,rbt0_y_est)
        rbt1_r = distnce(0,0,rbt1_x_est,rbt1_y_est)
        rbt2_r = distnce(0,0,rbt2_x_est,rbt2_y_est)
        rbt3_r = distnce(0,0,rbt3_x_est,rbt3_y_est)
        Rref = 3

        message_rbt0 = Twist()
        vxrbt0 = vx_ref(0,estposrbt0,rbt_pos,rbt0_phi,rbt0_psi_est)
        wzrbt0 = wz_ref(Rref,rbt0_r,rbt0_phi,rbt0_psi_est)
        message_rbt0.linear.x = vxrbt0
        message_rbt0.angular.z = wzrbt0

        message_rbt1 = Twist()
        vxrbt1 = vx_ref(1,estposrbt1,rbt_pos,rbt1_phi,rbt1_psi_est)
        wzrbt1 = wz_ref(Rref,rbt1_r,rbt1_phi,rbt1_psi_est)
        message_rbt1.linear.x = vxrbt1
        message_rbt1.angular.z = wzrbt1

        message_rbt2 = Twist()
        vxrbt2 = vx_ref(2,estposrbt2,rbt_pos,rbt2_phi,rbt2_psi_est)
        wzrbt2 = wz_ref(Rref,rbt2_r,rbt2_phi,rbt2_psi_est)
        message_rbt2.linear.x = vxrbt2
        message_rbt2.angular.z = wzrbt2

        message_rbt3 = Twist()
        vxrbt3 = vx_ref(3,estposrbt3,rbt_pos,rbt3_phi,rbt3_psi_est)
        wzrbt3 = wz_ref(Rref,rbt3_r,rbt3_phi,rbt3_psi_est)
        message_rbt3.linear.x = vxrbt3
        message_rbt3.angular.z = wzrbt3

        self.publisher_0.publish(message_rbt0)
        self.publisher_1.publish(message_rbt1)
        self.publisher_2.publish(message_rbt2)
        self.publisher_3.publish(message_rbt3)
        self.i0 += 1.0
        timestep_cnt += 1.0
        print('timestep : %f' %(timestep_cnt))
        print('Recieved rbt0 - pos_x err : %f, pos_y err : %f, r : %f' % (rbt0_x_est - rbt0_x_act, rbt0_y_est - rbt0_y_act, rbt0_r))
        print('Recieved rbt1 - pos_x err : %f, pos_y err : %f, r : %f' % (rbt1_x_est - rbt1_x_act, rbt1_y_est - rbt1_y_act, rbt1_r))
        print('Recieved rbt2 - pos_x err : %f, pos_y err : %f, r : %f' % (rbt2_x_est - rbt2_x_act, rbt2_y_est - rbt2_y_act, rbt2_r))
        print('Recieved rbt3 - pos_x err : %f, pos_y err : %f, r : %f' % (rbt3_x_est - rbt3_x_act, rbt3_y_est - rbt3_y_act, rbt3_r))

    ######################################################################

def RPSS(rbt0,rbti,rbt_pose,is_faulty):
    if is_faulty == 1:
        delx = rbt_pose[rbti][0] - rbt_pose[rbt0][0] + random.choice([4,-4])
        dely = rbt_pose[rbti][1] - rbt_pose[rbt0][1] + random.choice([4,-4])
        delpsi = wrap2pi(rbt_pose[rbti][2] - rbt_pose[rbt0][2] + random.choice([4,-4])*3*(math.pi/180))
    else:
        delx = rbt_pose[rbti][0] - rbt_pose[rbt0][0]
        dely = rbt_pose[rbti][1] - rbt_pose[rbt0][1]
        delpsi = wrap2pi(rbt_pose[rbti][2] - rbt_pose[rbt0][2])
    del_pose = numpy.array([delx,dely,delpsi])
    return del_pose
def RPSS_trgt(rbt0,rbt_pose,is_faulty):
    if is_faulty == 1:
        delx = 0.0 - rbt_pose[rbt0][0] + random.choice([4,-4])
        dely = 0.0 - rbt_pose[rbt0][1] + random.choice([4,-4])
        delpsi = wrap2pi(0.0 - rbt_pose[rbt0][2] + random.choice([4,-4])*3*(math.pi/180))
    else:
        delx = 0.0 - rbt_pose[rbt0][0]
        dely = 0.0 - rbt_pose[rbt0][1]
        delpsi = wrap2pi(0.0 - rbt_pose[rbt0][2])
    del_pose = numpy.array([delx,dely,delpsi])
    return del_pose
def RPSS_tgt(rbt0,rbt_pose,is_faulty):
    if is_faulty == 1:
        delx = (0.0 - rbt_pose[rbt0][0] + random.choice([4,-4]))*(-1.0)
        dely = (0.0 - rbt_pose[rbt0][1] + random.choice([4,-4]))*(-1.0)
        delpsi = wrap2pi((0.0 - rbt_pose[rbt0][2] + random.choice([4,-4])*3*(math.pi/180))*(-1.0))
    else:
        delx = (0.0 - rbt_pose[rbt0][0])*(-1.0)
        dely = (0.0 - rbt_pose[rbt0][1])*(-1.0)
        delpsi = wrap2pi((0.0 - rbt_pose[rbt0][2])*(-1.0))
    return delx,dely,delpsi

def rand_comm(G):
    Gmid = G - numpy.diag(numpy.ones(4))
    m = numpy.random.choice([0,1],size = (4,4))
    m1 = numpy.maximum(m,numpy.transpose(m))
    G_new = numpy.multiply(Gmid,m1) + numpy.diag(numpy.ones(4))
    return G_new
def neighID(rbt,Gn):
    l = numpy.array(Gn[rbt][:])
    neighID = numpy.where(l == 1)[0]
    return neighID
def f(xs,ys):
    w = xs - ys
    return w
def distnce(x0,y0,x1,y1):
    r = math.sqrt(math.pow(x0-x1,2) + math.pow(y0-y1,2))
    return r
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z # in radians

def wrap2pi(theta):
    theta_1 = theta
    if abs(theta_1) >= 2*math.pi:
        theta_1 = math.remainder(theta_1,2*math.pi)
    if theta_1 >= math.pi:
        theta_new = -2*math.pi + theta
    elif theta_1 < -math.pi:
        theta_new = 2*math.pi + theta
    else:
        theta_new = theta_1
    return theta_new

def signf(x):
    if x > 0:
        out = 1
    elif x < 0:
        out = -1
    else:
        out = 0
    return out

def wz_ref(Rref,r,phi,psi):
    kw1 = 1.0
    wzref = kw1*wrap2pi((wrap2pi((math.pi)/2 + wrap2pi(phi) + wrap2pi(phi_crr(Rref,r,phi,psi))) - wrap2pi(psi)))
    return wzref
def phi_crr(Rref,r,phi,psi):
    if abs(Rref - r) > 0.4:
        kw2 = 0.6
    else:
        kw2 = 0.3
    phicrr = kw2*signf(Rref-r)*(math.cos(psi)*math.sin(phi) - math.cos(phi)*math.sin(psi))
    return phicrr
def vx_ref(rbtID,rbtID_pos,rbt_pos,phi,psi):
    kv1 = 2.0
    x = rbtID_pos[0]
    y = rbtID_pos[1]
    xCA = rbt_pos[rbtID][0]
    yCA = rbt_pos[rbtID][1]
    xcn,ycn = neigh_dist(rbtID,rbt_pos)
    ndist = distnce(xCA,yCA,xcn,ycn)
    v_cnst = 0.4
    vref = v_cnst - kv1*((xcn-x)*math.cos(psi) + (ycn-y)*math.sin(psi))/(pow(ndist,3))
    return vref
def neigh_dist(rbtID,rbt_pos):
    dist = [0]*4
    for i in range(4):
        dist[i] = vec_dist(rbt_pos[rbtID][:],rbt_pos[i][:])
        if i == rbtID:
            dist[i] = 100000.0
    mindist = min(dist)
    minID = dist.index(mindist)
    minVec = rbt_pos[minID][:]
    return minVec
def vec_dist(vec1,vec2):
    dist = math.sqrt(pow(vec1[0]-vec2[0],2)+pow(vec1[1]-vec2[1],2))
    return dist
def neigh2ID(rbt0,rbt1,Gn):
    l = numpy.array(Gn[rbt0][:])
    m = numpy.array(Gn[rbt1][:])
    n = numpy.multiply(l,m)
    neigh2ID = numpy.where(n == 1)[0]
    return neigh2ID

##############################

def DLDCL1hop_pos(t,rbt,Gn,L,neigh_pose_IMU,neigh_pose_PRJ,neigh_delposrbt,neigh_delpostgt):
    To = 200.0
    st = t % To
    if math.ceil(t/To) == math.floor(t/To):
        L = {}
    for p in [0,1]: # 0: IMU, 1: PRJ
        for j in neighID(rbt,Gn):
            if L.get((p,j)) == None:
                L[p,j] = 0.0
        if L.get(p) == None:
            L[p] = 0.0

    eta_w = 2.0
    eta_g = 2.0

    ghat = numpy.zeros(shape=(2))
    what = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            what[p,ij] = math.exp(-eta_w*L[(p,j)]*st)
            ij += 1
        ghat[p] = math.exp(-eta_g*L[p]*st)

    mlprc = math.pow(10,-40)
    what,ghat = nrmlzWgts1hop(rbt,Gn,what,ghat,mlprc)

    gt = numpy.zeros(shape=(2))
    wt = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            wt[p,ij] = (what[p,ij])/sum(what[p,:])
            ij += 1
        gt[p] = (ghat[p])/sum(ghat[:])

    x_g = numpy.zeros(shape=(2,2))
    x_w = numpy.zeros(shape=(2,len(neighID(rbt,Gn)),2))
    for p in [0,1]:
        ij = 0 # where does j stand among the immediate neighbors of rbt
        for j in neighID(rbt,Gn):
            if p == 0:
                x_w[p,ij,:] = neigh_pose_IMU[ij,0:2] + neigh_delpostgt[ij]
            else:
                x_w[p,ij,:] = neigh_pose_PRJ[ij,0:2] + neigh_delpostgt[ij]
            ij += 1
        wt_arr = numpy.transpose(numpy.array([wt[p,:],wt[p,:]]))
        x_g[p,:] = sum(numpy.multiply(wt_arr,x_w[p,:,:]))
    x_est_tgt = (gt[0])*x_g[0,:] + (gt[1])*x_g[1,:]
    #print(x_est_tgt)
    # cumulative loss update
    tgt_pos = numpy.zeros(shape=(2))
    for p in [0,1]: # 0: IMU, 1: PRJ
        ij = 0
        for j in neighID(rbt,Gn):
            L[p,j] = (L[p,j]*st + lossfn_pos(tgt_pos,x_w[p,ij,:]))/(st+1)
            ij += 1
        L[p] = (L[p]*st + lossfn_pos(tgt_pos,x_g[p,:]))/(st+1)

    #

    ghat = numpy.zeros(shape=(2))
    what = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            what[p,ij] = math.exp(-eta_w*L[p,j]*(st+1))
            ij += 1
        ghat[p] = math.exp(-eta_g*L[p]*(st+1))

    #mlprc = math.pow(10,-40)
    what,ghat = nrmlzWgts1hop(rbt,Gn,what,ghat,mlprc)
    #print(ahat)
    gt = numpy.zeros(shape=(2))
    wt = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            wt[p,ij] = (what[p,ij])/sum(what[p,:])
            ij += 1
        gt[p] = (ghat[p])/sum(ghat[:])

    # estimation phase
    xe_g = numpy.zeros(shape=(2,2))
    xe_w = numpy.zeros(shape=(2,len(neighID(rbt,Gn)),2))
    for p in [0,1]:
        ij = 0 # where does j stand among the immediate neighbors of rbt
        for j in neighID(rbt,Gn):
            ji_idx = int(numpy.where(neighID(j,Gn) == rbt)[0]) # where does rbt stand among the immediate neighbors of j
            if p == 0:
                xe_w[p,ij,:] = neigh_pose_IMU[ij,0:2] + neigh_delposrbt[ij][ji_idx]
            else:
                xe_w[p,ij,:] = neigh_pose_PRJ[ij,0:2] + neigh_delposrbt[ij][ji_idx]
            ij += 1
        wt_arr = numpy.transpose(numpy.array([wt[p,:],wt[p,:]]))
        xe_g[p,:] = sum(numpy.multiply(wt_arr,xe_w[p,:,:]))
    x_est_rbt = (gt[0])*xe_g[0,:] + (gt[1])*xe_g[1,:]
    return x_est_rbt[0],x_est_rbt[1],L

def DLDCL1hop_psi(t,rbt,Gn,L,neigh_pose_IMU,neigh_pose_PRJ,neigh_delposrbt,neigh_delpostgt):
    To = 200.0
    st = t % To
    if math.ceil(t/To) == math.floor(t/To):
        L = {}
    for p in [0,1]: # 0: IMU, 1: PRJ
        for j in neighID(rbt,Gn):
            if L.get((p,j)) == None:
                L[p,j] = 0.0
        if L.get(p) == None:
            L[p] = 0.0
    print(st)
    eta_w = 2.0
    eta_g = 2.0

    ghat = numpy.zeros(shape=(2))
    what = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            what[p,ij] = math.exp(-eta_w*L[(p,j)]*st)
            ij += 1
        ghat[p] = math.exp(-eta_g*L[p]*st)

    mlprc = math.pow(10,-40)
    what,ghat = nrmlzWgts1hop(rbt,Gn,what,ghat,mlprc)

    gt = numpy.zeros(shape=(2))
    wt = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            wt[p,ij] = (what[p,ij])/sum(what[p,:])
            ij += 1
        gt[p] = (ghat[p])/sum(ghat[:])

    x_g = numpy.zeros(shape=(2))
    x_w = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0 # where does j stand among the immediate neighbors of rbt
        for j in neighID(rbt,Gn):
            if p == 0:
                x_w[p,ij] = wrap2pi(neigh_pose_IMU[ij,2] + neigh_delpostgt[ij])
            else:
                x_w[p,ij] = wrap2pi(neigh_pose_PRJ[ij,2] + neigh_delpostgt[ij])
            ij += 1
        wt_arr = numpy.array(wt[p,:])
        x_g[p] = wrap2pi(sum(numpy.multiply(wt_arr,x_w[p,:])))
    x_est_tgt = wrap2pi((gt[0])*x_g[0] + (gt[1])*x_g[1])

    # cumulative loss update
    tgt_pos = numpy.zeros(shape=(1))
    for p in [0,1]: # 0: IMU, 1: PRJ
        ij = 0
        for j in neighID(rbt,Gn):
            L[p,j] = (L[p,j]*st + lossfn_psi(tgt_pos,x_w[p,ij]))/(st+1)
            ij += 1
        L[p] = (L[p]*st + lossfn_psi(tgt_pos,x_g[p]))/(st+1)

    #

    ghat = numpy.zeros(shape=(2))
    what = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            what[p,ij] = math.exp(-eta_w*L[p,j]*(st+1))
            ij += 1
        ghat[p] = math.exp(-eta_g*L[p]*(st+1))


    what,ghat = nrmlzWgts1hop(rbt,Gn,what,ghat,mlprc)

    gt = numpy.zeros(shape=(2))
    wt = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0
        for j in neighID(rbt,Gn):
            wt[p,ij] = (what[p,ij])/sum(what[p,:])
            ij += 1
        gt[p] = (ghat[p])/sum(ghat[:])


    # estimation phase
    xe_g = numpy.zeros(shape=(2))
    xe_w = numpy.zeros(shape=(2,len(neighID(rbt,Gn))))
    for p in [0,1]:
        ij = 0 # where does j stand among the immediate neighbors of rbt
        for j in neighID(rbt,Gn):
            ji_idx = int(numpy.where(neighID(j,Gn) == rbt)[0]) # where does rbt stand among the immediate neighbors of j
            if p == 0:
                xe_w[p,ij] = wrap2pi(neigh_pose_IMU[ij,2] + neigh_delposrbt[ij][ji_idx])
            else:
                xe_w[p,ij] = wrap2pi(neigh_pose_PRJ[ij,2] + neigh_delposrbt[ij][ji_idx])
            ij += 1
        wt_arr = numpy.array(wt[p,:])
        xe_g[p] = wrap2pi(sum(numpy.multiply(wt_arr,xe_w[p,:])))
    x_est_rbt = wrap2pi((gt[0])*xe_g[0] + (gt[1])*xe_g[1])
    return x_est_rbt,L

def nrmlzWgts1hop(rbt,Gn,what,ghat,delt):
    if max(ghat[:]) <= delt:
        for p in [0,1]:
            ghat[p] = (ghat[p])/max(ghat[:])
    else:
        for p in [0,1]:
            if ghat[p] <= delt:
                ghat[p] = 0.0
    for p in [0,1]:
        if max(what[p,:]) <= delt:
            ij = 0
            for j in neighID(rbt,Gn):
                what[p,ij] = (what[p,ij])/max(what[p,:])
                ij += 1
        else:
            ij = 0
            for j in neighID(rbt,Gn):
                if what[p,ij] <= delt:
                    what[p,ij] = 0.0
                ij += 1
    return what,ghat

#####################################

def lossfn_pos(tgt_pos,est_pos): # in meters
    delpos = abs(tgt_pos - est_pos)
    loss_val = math.sqrt(numpy.dot(delpos,delpos))/(10.0)
    lss_sat  = min(loss_val,1.0)
    return lss_sat
def lossfn_psi(tgt_psi,est_psi): # in meters
    delpsi = abs(wrap2pi(tgt_psi - est_psi))
    loss_val = (delpsi)/(15*math.pi/180)
    lss_sat  = min(loss_val,1.0)
    return lss_sat
def nrmlzWgts(rbt,Gn,ahat,bhat,what,ghat,delt):
    if max(ghat[:]) <= delt:
        for p in [0,1]:
            ghat[p] = (ghat[p])/max(ghat[:])
    else:
        for p in [0,1]:
            if ghat[p] <= delt:
                ghat[p] = 0.0
    for p in [0,1]:
        if max(what[p,:]) <= delt:
            ij = 0
            for j in neighID(rbt,Gn):
                what[p,ij] = (what[p,ij])/max(what[p,:])
                ij += 1
        else:
            ij = 0
            for j in neighID(rbt,Gn):
                if what[p,ij] <= delt:
                    what[p,ij] = 0.0
                ij += 1
        ij = 0
        for j in neighID(rbt,Gn):
            if max(bhat[p][ij][:]) <= delt:
                ik = 0
                for k in neigh2ID(rbt,j,Gn):
                    bhat[p][ij][ik] = (bhat[p][ij][ik])/max(bhat[p][ij][:])
                    ik += 1
            else:
                ik = 0
                for k in neigh2ID(rbt,j,Gn):
                    if bhat[p][ij][ik] <= delt:
                        bhat[p][ij][ik] = 0.0
                    ik += 1
            ik = 0
            for k in neigh2ID(rbt,j,Gn):
                if max(ahat[p][ij][ik,:]) <= delt:
                    for i3 in [0,1]:
                        ahat[p][ij][ik,i3] = (ahat[p][ij][ik,i3])/max(ahat[p][ij][ik,:])
                else:
                    for i3 in [0,1]:
                        if ahat[p][ij][ik,i3] <= delt:
                            ahat[p][ij][ik,i3] = 0.0
                ik += 1
            ij += 1
    return ahat,bhat,what,ghat
def prjphse(vxrbt,wzrbt,rbtpos,rbtpsi,delT):
    x = rbtpos[0]
    y = rbtpos[1]
    psi = rbtpsi
    x_prj = x + vxrbt*math.cos(psi)*(delT)
    y_prj = y + vxrbt*math.sin(psi)*(delT)
    psi_prj = psi + wzrbt*(delT)
    pos_prj = numpy.array([x_prj, y_prj])
    pose_prj = numpy.array([x_prj, y_prj, psi_prj])
    return pose_prj

###############################################################

def main(args=None):
    global L0_pos,L1_pos,L2_pos,L3_pos
    global L0_psi,L1_psi,L2_psi,L3_psi
    global timestep_cnt
    global vxrbt0, vxrbt1, vxrbt2, vxrbt3
    global wzrbt0, wzrbt1, wzrbt2, wzrbt3
    global estposrbt0, estposrbt1, estposrbt2, estposrbt3
    global estpsirbt0, estpsirbt1, estpsirbt2, estpsirbt3
    global deltaT
    deltaT = 0.1 # seconds
    L0_pos = {}
    L1_pos = {}
    L2_pos = {}
    L3_pos = {}
    L0_psi = {}
    L1_psi = {}
    L2_psi = {}
    L3_psi = {}
    timestep_cnt = 0.0
    vxrbt0 = 0.0
    vxrbt1 = 0.0
    vxrbt2 = 0.0
    vxrbt3 = 0.0
    wzrbt0 = 0.0
    wzrbt1 = 0.0
    wzrbt2 = 0.0
    wzrbt3 = 0.0
    estposrbt0 = numpy.zeros(shape=(2))
    estposrbt1 = numpy.zeros(shape=(2))
    estposrbt2 = numpy.zeros(shape=(2))
    estposrbt3 = numpy.zeros(shape=(2))
    estpsirbt0 = 0.0
    estpsirbt1 = 0.0
    estpsirbt2 = 0.0
    estpsirbt3 = 0.0

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    #rclpy.shutdown()

if __name__ == '__main__':
    main()
