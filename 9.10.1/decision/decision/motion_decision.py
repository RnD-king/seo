import rclpy

from robot_msgs.msg import MotionCommand, LineResult, FallResult, BallResult, MotionEnd
from rclpy.node import Node

class Motion:
    Forward_Four = 1
    Turn_Left = 2
    Turn_Right = 3
    Back = 4
    Back_Half = 5
    Forward_Half = 6
    Forward_Left = 7
    Forward_Right = 8
    Pick = 9
    Re_Catch = 10
    Catch_Finish = 11
    Forward_One = 12
    Out_Right = 13
    Out_Left = 14
    Left_Half = 15
    Right_Half = 16
    Shoot_Ready = 17
    Shoot = 18
    Shoot_Finish = 19
    Recovery = 77
    Stop = 99

class MotionDecision(Node):
    def __init__(self):
        super().__init__('line_result_subscribe')

        #초기값 설정
        self.fall_detect = False
        self.ball_on = False
        self.line_on = False
        self.motion_end = False

        # self.motion_during = False  

        self.res = 0

        #라인 결과 subscribe
        self.line_result_subscribe = self.create_subscription( 
            LineResult,
            '/line_result', 
            self.LineResultCallback,
            10
        )

        #fall 상태 subscribe
        self.fall_subscribe = self.create_subscription(FallResult, '/fall_result', self.FallResultCallback, 10)

        #원하는 motion publish
        self.motion_publish = self.create_publisher(MotionCommand, '/motion_command', 10)

        # ball 관련 subscribe
        self.ball_subscribe = self.create_subscription(BallResult, '/ball_result', self.BallCallback, 10)

        # motion_end subscibe
        self.motion_end_sub = self.create_subscription(MotionEnd, '/motion_end', self.MotionEndCallback, 10)

    # pick motion이 끝나면 pick_end를 받아오기
    def MotionEndCallback(self, end_msg: MotionEnd):
        self.motion_end = end_msg.motion_end_detect
        # self.motion_during = end_msg.motion_during
        self.get_logger().info(f"motion_end: {end_msg.motion_end_detect}")

    def BallCallback(self, Ball_msg: BallResult):
        # if(self.motion_during):
        if(self.motion_end):
            if(self.ball_on == False):
                self.ball_res = Ball_msg.res
                self.ball_on = True
                self.ResultUnion()
            else:
                self.get_logger().info(f"아직 union 안함1111")
        else:
                self.get_logger().info(f"아직 union 안함2222")

    def FallResultCallback(self, fall_msg: FallResult):
        self.get_logger().info(f"[FallResult] yaw: {fall_msg.prev_yaw_deg}, state: {fall_msg.fall_detect}")
        self.fall_detect = fall_msg.fall_detect
        self.ResultUnion()

    def LineResultCallback(self, line_msg: LineResult):
        # ball을 감지하면 라인 안 받아오게
        # if(self.motion_during):
        if(self.motion_end):
            if(self.line_on == False):
                self.get_logger().info(f"[LineResult] res: {line_msg.res}, angle: {line_msg.angle}")
                self.line_res = line_msg.res
                self.angle = line_msg.angle
                self.line_on = True
                self.ResultUnion()
            else:
                self.get_logger().info(f"아직 union 안함333")
        else:
                self.get_logger().info(f"아직 union 안함4444")
            
    

    def ResultUnion(self):
        if(self.ball_on == True & self.line_on == True):
            if(self.ball_res != 99):
                self.res = self.ball_res
                self.angle = 0
                self.MotionResult()
                self.get_logger().info(f"ballressssssssssssssssssssssssss") 
                return
            self.res = self.line_res
            self.MotionResult()




    # subscribe한 fall_msg와 line_msg를 이용한 motion 결정
    def MotionResult(self):
        
        motion_msg = MotionCommand()
  
        if self.fall_detect:
            motion_msg.command = Motion.Stop

        elif self.res == 1:
            motion_msg.command = Motion.Forward_Four

        elif self.res == 2:
            motion_msg.command = Motion.Turn_Left
        
        elif self.res == 3:
            motion_msg.command = Motion.Turn_Right
        
        elif self.res == 4:   
            motion_msg.command = Motion.Back

        elif self.res == 5:   
            motion_msg.command = Motion.Back_Half

        elif self.res == 6:   
            motion_msg.command = Motion.Forward_Half

        elif self.res == 7:
            motion_msg.command = Motion.Forward_Left

        elif self.res == 8:
            motion_msg.command = Motion.Forward_Right

        elif self.res == 9:
            motion_msg.command = Motion.Pick

        elif self.res == 10:
            motion_msg.command = Motion.Re_Catch # 공 다시 잡기

        elif self.res == 11:
            motion_msg.command = Motion.Catch_Finish

        elif self.res == 12:
            motion_msg.command = Motion.Forward_One

        elif self.res == 13:
            motion_msg.command = Motion.Out_Right # 왼쪽으로 돌고 1step
            
        elif self.res == 14:
            motion_msg.command = Motion.Out_Left # 오른쪽으로 돌고 1step

        elif self.res == 15:
            motion_msg.command = Motion.Left_Half 
        
        elif self.res == 16:
            motion_msg.command = Motion.Right_Half 

        elif self.res == 17:
            motion_msg.command = Motion.Shoot_Ready
    
        elif self.res == 18:
            motion_msg.command = Motion.Shoot 

        elif self.res == 19:
            motion_msg.command = Motion.Shoot_Finish

        elif self.res == 77:
            motion_msg.command = Motion.Recovery

        elif self.res == 99:
            motion_msg.command = Motion.Stop
        
        motion_msg.angle = self.angle

    
        self.motion_publish.publish(motion_msg)
        self.get_logger().info(f"Published motion command: {motion_msg.command}")

        self.ball_on = False
        self.line_on = False
        self.motion_end = False
        # self.motion_during = True


def main(args=None):
    rclpy.init(args=args)
    node = MotionDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
