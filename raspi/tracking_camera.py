# First import the library
import pyrealsense2 as rs
import math

def BYTE0(data):
    return (data & 0xff)
def BYTE1(data):
    return ((data >> 8) & 0xff)

class T265:
    def __init__(self):
        # t265 status
        self.status = 0
        # 实例化相机对象
        self.pipe = rs.pipeline()
        # 配置传输位姿数据
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        # 声明成员
        self.pos = dict()
        self.vel = dict()
        self.qua = dict()
        # 开启 T265 位姿数据传输
        self.pipe.start(cfg)

    def get_frame(self):
        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        # 获取位姿数据
        if pose:
            pose_data = pose.get_pose_data()
            return pose_data
        else:
            return 0

    # 获取相对起始点位置
    def get_position(self, pose_data):
        self.pos['x'] = -pose_data.translation.z
        self.pos['y'] = -pose_data.translation.x
        self.pos['z'] = pose_data.translation.y

    # 获取速度
    def get_velocity(self, pose_data):
        self.vel['x'] = -pose_data.velocity.z
        self.vel['y'] = -pose_data.velocity.x
        self.vel['z'] = pose_data.velocity.y

    # 获取四元素并转换成欧拉角
    def get_quaternion(self, pose_data):
        self.qua['w'] = pose_data.rotation.w
        self.qua['x'] = -pose_data.rotation.z
        self.qua['y'] = pose_data.rotation.x
        self.qua['z'] = -pose_data.rotation.y
        self.yaw_theta = math.atan2(2.0 * (self.qua['w']*self.qua['z'] + self.qua['x']*self.qua['y']), 
                                    self.qua['w']*self.qua['w'] + self.qua['x']*self.qua['x'] - 
                                    self.qua['y']*self.qua['y'] - self.qua['z']*self.qua['z']) * 180.0 / math.pi

    # 添加发送数据
    def add_send_data(self):
        check_sum1 = check_sum2 = 0
        # t265 转发给飞控的帧 ID 是 01
        frame = [0xbb,0x01,23,BYTE0(int(self.pos['x'] * 100)),BYTE1(int(self.pos['x'] * 100)),
                       BYTE0(int(self.pos['y'] * 100)),BYTE1(int(self.pos['y'] * 100)),
                       BYTE0(int(self.pos['z'] * 100)),BYTE1(int(self.pos['z'] * 100)),
                       BYTE0(int(self.vel['x'] * 100)),BYTE1(int(self.vel['x'] * 100)),
                       BYTE0(int(self.vel['y'] * 100)),BYTE1(int(self.vel['y'] * 100)),
                       BYTE0(int(self.vel['z'] * 100)),BYTE1(int(self.vel['z'] * 100)),
                       BYTE0(int(self.yaw_theta * 100)),BYTE1(int(self.yaw_theta * 100)),
                       BYTE0(int(self.qua['w'] * 10000)),BYTE1(int(self.qua['w'] * 10000)),
                       BYTE0(int(self.qua['x'] * 10000)),BYTE1(int(self.qua['x'] * 10000)),
                       BYTE0(int(self.qua['y'] * 10000)),BYTE1(int(self.qua['y'] * 10000)),
                       BYTE0(int(self.qua['z'] * 10000)),BYTE1(int(self.qua['z'] * 10000)),
                       self.status]
        for i in frame:
            check_sum1 += i
            check_sum2 += check_sum1
        frame += [BYTE0(check_sum1)]
        frame += [BYTE0(check_sum2)]

        tx_data = bytes(frame)        

        return tx_data
    