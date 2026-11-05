import rclpy
from threading import Thread

#from my_robot_commands.controller_v2 import RobotControllerNode
from my_robot_commands.controller_v3 import RobotControllerNode

# ==========================================
# POINT D'ENTRÉE DU SYSTÈME
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    
    controller_node = RobotControllerNode()
    
    # Threading des entrées/sorties ROS2 pour ne pas bloquer l'UI principale
    ros_thread = Thread(target=rclpy.spin, args=(controller_node,), daemon=True)
    ros_thread.start()
    
    try:
        controller_node.view.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
