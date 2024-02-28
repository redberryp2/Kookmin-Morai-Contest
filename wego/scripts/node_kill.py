import subprocess
import rospy

def get_running_nodes():
    # 현재 실행 중인 노드들의 이름을 얻기
    result = subprocess.check_output(["rosnode", "list"])
    return result.split()

def kill_all_nodes():
    # 모든 노드 종료
    subprocess.call(["rosnode", "kill", node])

if __name__ == "__main__":
    rospy.init_node("node_killer")

    try:
        kill_all_nodes()
    except rospy.ROSInterruptException:
        pass
