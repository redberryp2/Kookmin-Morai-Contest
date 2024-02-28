import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
import subprocess  # subprocess 모듈을 임포트
from wego.msg import parking 

class navigation_client():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cam_pub = rospy.Publisher('/parking', parking, queue_size =10)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 17.916861210483123  # 예시 좌표
        self.goal.target_pose.pose.position.y = -10.1992938760294  # 예시 좌표
        # self.goal.target_pose.pose.position.x = 19.916861210483123  # 예시 좌표
        # self.goal.target_pose.pose.position.y = -10.1992938760294  # 예시 좌표

        self.goal.target_pose.pose.orientation.w = 1.0
        self.goal.target_pose.pose.orientation.z = 0.0

        self.goal_sent = False
        self.start_time = None  # start_time 초기화
        self.parking = parking()
        self.isparking = False

    def run(self):
        if not self.goal_sent:
            self.client.send_goal(self.goal)
            self.goal_sent = True
            self.start_time = rospy.Time.now()  # 목표를 보낼 때 현재 시간으로 start_time 업데이트
        else:
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached")
                self.isparking = True
        self.parking.parking_flag = self.isparking
        self.cam_pub.publish(self.parking.parking_flag)
                

                


def main():
    rospy.init_node('navigation_client')
    nc = navigation_client()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()

if __name__ == '__main__':
    main()




