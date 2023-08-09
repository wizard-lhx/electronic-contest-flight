import communicate
import tracking_camera
import threading
import lgpio as lg

# 开启串口0，lgpio 库给的 serial_open() 函数的例子有误，第一个参数没有 ‘/dev/’ 
uart0 = communicate.UART('ttyAMA1', 230400)
h = lg.gpiochip_open(0)
lg.gpio_claim_output(h,23,lg.LOW)
# 结束进程标志，方便从主进程中结束子线程
thread_stop_flag = False

# 接受 t265 的位姿数据并转发给飞控的任务
def t265_task():
    try:
        # 建立 t265 对象
        t265 = tracking_camera.T265()

        # 获取位姿数据
        while not thread_stop_flag:
            pose_data = t265.get_frame()
            if pose_data:
                t265.get_position(pose_data)
                t265.get_velocity(pose_data)
                t265.get_quaternion(pose_data)
                t265.status = 1 # t265 正常
                lg.gpio_write(h,23,lg.HIGH)
                # 转发数据给飞控               
                uart0.rpi_write_byte(t265.add_send_data())
    except:
        print("T265 start failed!")
        status = 0 # t265 异常(因为 t265 对象可能创建失败，所以不能再用 t265 对象)
        lg.gpio_write(h,23,lg.LOW)
        # 转发数据给飞控               
        uart0.rpi_write_byte(bytes([0xbb,0x01,0x23,0,0,0,0,0,0,
                                    0,0,0,0,0,0,
                                    0,0,
                                    0,0,0,0,0,0,0,0,status,223,95]))

# 串口接受飞控数据任务
def command_receive_task():
    while not thread_stop_flag:
        data = uart0.rpi_read_byte()
        if data >= 0:
            # 解析接受的数据
            communicate.rpi_receive_data(data)
        #print(communicate.cmd)

# 使用多线程
thread_send_t265_data = threading.Thread(target=t265_task)
thread_receive_lx_data = threading.Thread(target=command_receive_task)
try:
    thread_send_t265_data.start()
    thread_receive_lx_data.start()
    while not thread_stop_flag:
        pass
# 方便调试时终止所有线程
except KeyboardInterrupt:
    thread_stop_flag = True
