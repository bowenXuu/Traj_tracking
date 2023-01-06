
import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PointStamped,Pose
from nav_msgs.msg import Odometry
from ros_numpy import msgify, numpify
from sensor_msgs.msg import PointCloud2,Imu
from sensor_msgs import point_cloud2
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory,MultiDOFJointTrajectoryPoint
from sensor_msgs.msg import JointState
from ros_numpy import msgify, numpify

pc_topic=" "
joint_topic=" "
md_traj_topic="/hummingbird/command/trajectory"
quad_traj_topic="ddd"

def inverse_of_trans(T):
    R=T[:3,:3]
    t=T[:3,3].T
    
    T_inv=T
    T_inv[:3,:3]=R.transpose()
    T_inv[:3,3]=-R.transpose().dot(t)
    
    return T_inv


class eyepulator_server(object):
    
    def __init__(self):
        rospy.init_node("eyepulator_server_node") 

        self.traj_sub=rospy.Subscriber("/hummingbird/command/trajectory",MultiDOFJointTrajectory,self.traj_callback,queue_size=10)
        self.joint_sub=rospy.Subscriber('/hummingbird/joint_states',JointState,self.joint_state_callback,queue_size=10)
        self.pose_sub=rospy.Subscriber("/hummingbird/ground_truth/pose",Pose, self.pose_callback,queue_size=10)
        self.whole_command_traj_sub=rospy.Subscriber("hummingbird/whole_command_trajectory",MultiDOFJointTrajectory,self.whole_traj_callback,queue_size=10)
      #  self.imu_sub=rospy.Subscriber('/imu/data',Imu,self.imu_callback,queue_size=19)

        self.jo5_cmd_pub=rospy.Publisher('/hummingbird/jo5/position_command',Float64,queue_size=10)
        self.jo6_cmd_pub=rospy.Publisher('/hummingbird/jo6/position_command',Float64,queue_size=10)

        
        self.poi_W=np.array([[1.6,-0.8,0,1]]).T

   

    def whole_traj_callback(self,traj_msg):
        print((traj_msg.points))

    def pose_callback(self,pose_msg):
        self.T_W_B=pose_msg
        
    def joint_state_callback(self,joint_state_msg):
        
        
        if joint_state_msg.name[0]=="joint1":

            joint_state_array=np.array(joint_state_msg.position)
           

            jo5=joint_state_array[4]
            jo6=joint_state_array[5]

            self.jo5=jo5
            self.jo6=jo6

            self.T_B_C=np.array([
                        [ np.cos(jo5)*np.cos(jo6),  np.sin(jo6)*np.cos(jo5), np.sin(jo5), 0],
                        [         -np.sin(jo6),           np.cos(jo6),        0, 0],
                        [-np.sin(jo5)*np.cos(jo6), -np.sin(jo5)*np.sin(jo6), np.cos(jo5), 0],
                        [                 0,                  0,        0, 1]])



    def traj_callback(self,multi_dof_traj_msg):
        #transform multiDOFtraj to quadrotor_msg_traj


        jo5_cmd_list=[]
        jo6_cmd_list=[]

        r=rospy.Rate(10)
        #print(len(multi_dof_traj_msg.points))

        for i in range(len(multi_dof_traj_msg.points)):

            
            T_W_B=numpify(multi_dof_traj_msg.points[i].transforms[0])
            T_B_W=inverse_of_trans(T_W_B)

            poi_B=T_B_W.dot(self.poi_W)

            print(-poi_B)

            poi_B=[1,-1,0,1]


            jo5=math.atan2(poi_B[2],poi_B[0])
            jo6=-math.atan2(poi_B[1],math.sqrt(poi_B[0]**2+poi_B[2]**2))

            jo1=math.atan2(poi_B[1],poi_B[0])
            jo5=-math.atan2(poi_B[2],math.sqrt(poi_B[0]**2+poi_B[1]**2))

            print(jo1)
            print(jo5)
            

            jo5_msg=Float64()
            jo6_msg=Float64()
            jo5_msg.data=jo5
            jo6_msg.data=jo6

            jo5_cmd_list.append(jo5)
            jo6_cmd_list.append(jo6)

            
            self.jo5_cmd_pub.publish(jo5_msg)
            self.jo6_cmd_pub.publish(jo6_msg)

            #r.sleep()


if __name__ == '__main__':

    es=eyepulator_server()

    r=rospy.Rate(10)

    while not rospy.is_shutdown():
        jot_msg=Float64()
        jot_msg.data=20.0
        es.jo5_cmd_pub.publish(jot_msg)


    rospy.spin()