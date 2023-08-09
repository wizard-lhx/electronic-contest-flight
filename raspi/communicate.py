import lgpio as lg

def BYTE0(data):
    return (data & 0xff)
def BYTE1(data):
    return ((data >> 8) & 0xff)

class UART:
    # 开启串口
    def __init__(self,tty,baud):
        self.huart = lg.serial_open(tty,baud)

    def rpi_read_byte(self):
        # 接收串口数据一定要先判断串口有无数据，否则会报错
        if lg.serial_data_available(self.huart):
            # 返回的数据不是 bytes 型，而是 int 型
            return lg.serial_read_byte(self.huart)
        else:
            return -1

    def rpi_write_byte(self, tx_buffer):
        lg.serial_write(self.huart, tx_buffer)

# 接受解析函数
rx_state = 0
data_len = 0
data_cnt = 0
rx_buffer = list()
cmd = 0
def receive_data_anl(buffer, len):
    global cmd
    check_sum1 = 0
    check_sum2 = 0
    # 判断数据长度
    if buffer[2] != (len - 5):
        return None
    for i in range(len - 2):
        check_sum1 += buffer[i]
        check_sum2 += check_sum1
    # 判断校验和
    if BYTE0(check_sum1) != buffer[len-2] or BYTE0(check_sum2) != buffer[len-1]:
        return None
    if buffer[0] != 0xbb:
        return None
    if buffer[1] == 0x01:
        if buffer[3] & 0x80:
            cmd = -((((buffer[3]) | buffer[4] << 8) - 1) ^ 0xffff)   
        else:
            cmd = buffer[3] << 8 | buffer[4]

# 串口接受函数，把接收到的数据先放在缓冲区内，再解析
def rpi_receive_data(data):
    global rx_buffer
    global rx_state
    global data_cnt
    global data_len
    if data == 0xbb and rx_state == 0:
        rx_buffer = [data]
        rx_state += 1
    elif rx_state == 1:
        rx_buffer += [data]
        rx_state += 1
    elif rx_state == 2:
        rx_buffer += [data]
        data_len = data
        rx_state += 1
    elif rx_state == 3 and data_cnt <= data_len - 1:
        rx_buffer += [data]
        data_cnt += 1
        if data_cnt >= data_len:
            data_cnt = 0
            rx_state = 4
    elif rx_state == 4:
        rx_state += 1
        rx_buffer += [data]
    elif rx_state == 5:
        rx_buffer += [data]
        rx_state = 0
        receive_data_anl(rx_buffer, data_len+5)
    else:
        rx_state = 0