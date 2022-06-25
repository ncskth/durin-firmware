################################
#GENERATED FILE DO NOT EDIT
################################

from enum import Enum
import struct

def scaledFloat_to_uint(value, scale):
    return value * scale

def uint_to_scaledFloat(value, scale):
    return value / scale

def packedFloat_to_uint(value, minValue, maxValue, size):
    intMax = (1 << size * 8) - 1
    if(value < minValue):
      return 0
    if(value > maxValue):
      return intMax
    ratio = (value - minValue) / (maxValue - minValue)
    return 1 + ((intMax - 2)) * ratio
  
def uint_to_packedFloat(value, minValue, maxValue, size):
    intMax = (1 << size * 8) - 1
    if(value <= 0):
      return minValue - 1.0
    if(value >= intMax):
      return maxValue + 1.0
    ratio = (value - 1) / (intMax - 2)
    return ratio * (maxValue - minValue) + minValue

class tof_status(Enum):
    valid = 0
    half_valid = 1
    not_valid = 2
    no_change = 3
class nodes(Enum):
    durin = 0
    control = 1
class fields(Enum):
    vel_x_mms = 0
    vel_y_mms = 1
    rot_degs = 2
    motor_1_mms = 3
    motor_2_mms = 4
    motor_3_mms = 5
    motor_4_mms = 6
    intensity = 7
    led_r = 8
    led_g = 9
    led_b = 10
    sensor_id = 11
    ip = 12
    port = 13
    rate_ms = 14
    node_id = 15
    main_ssid = 16
    main_password = 17
    frame_number = 18
    checksum = 19
    frame_length = 20
    data = 21
    tof_1 = 22
    tof_2 = 23
    tof_3 = 24
    tof_4 = 25
    tof_5 = 26
    tof_6 = 27
    tof_7 = 28
    tof_8 = 29
    charge_percent = 30
    battery_voltage_mv = 31
    ax = 32
    ay = 33
    az = 34
    gx = 35
    gy = 36
    gz = 37
    mx = 38
    my = 39
    mz = 40
    set_this_to_zero = 41
class messages(Enum):
    acknowledge = 0
    reject = 1
    power_off = 2
    move_rob_centric = 3
    move_wheels = 4
    set_buzzer = 5
    set_led = 6
    poll_all = 7
    poll_sensor = 8
    start_stream = 9
    stop_stream = 10
    set_node_id = 11
    wifi_config = 12
    ota_packet = 13
    tof_12 = 14
    tof_34 = 15
    tof_56 = 16
    tof_78 = 17
    misc = 18
    uwb_distance = 19
class categories(Enum):
    none = 0
class acknowledge_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.acknowledge
        self._category = categories.none
        self._id = 0
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class reject_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.reject
        self._category = categories.none
        self._id = 127
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class power_off_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.power_off
        self._category = categories.none
        self._id = 1
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class move_rob_centric_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.move_rob_centric
        self._category = categories.none
        self._id = 2
        self._size = 6
        self._vel_x_mms = 0
        self._vel_y_mms = 0
        self._rot_degs = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_vel_x_mms(self, value):
        self._vel_x_mms = value
    def set_vel_y_mms(self, value):
        self._vel_y_mms = value
    def set_rot_degs(self, value):
        self._rot_degs = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<h", self._vel_x_mms)
        buf += struct.pack("<h", self._vel_y_mms)
        buf += struct.pack("<h", self._rot_degs)
        return buf
    def get_vel_x_mms(self):
        return self._vel_x_mms
    def get_vel_y_mms(self):
        return self._vel_y_mms
    def get_rot_degs(self):
        return self._rot_degs
    def get_all_data(self):
        data = []
        data.append((fields.vel_x_mms, self.get_vel_x_mms()))
        data.append((fields.vel_y_mms, self.get_vel_y_mms()))
        data.append((fields.rot_degs, self.get_rot_degs()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._vel_x_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        self._vel_y_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        self._rot_degs = struct.unpack_from("<h", buf, index)[0]
        index += 2
        return
class move_wheels_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.move_wheels
        self._category = categories.none
        self._id = 3
        self._size = 8
        self._motor_1_mms = 0
        self._motor_2_mms = 0
        self._motor_3_mms = 0
        self._motor_4_mms = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_motor_1_mms(self, value):
        self._motor_1_mms = value
    def set_motor_2_mms(self, value):
        self._motor_2_mms = value
    def set_motor_3_mms(self, value):
        self._motor_3_mms = value
    def set_motor_4_mms(self, value):
        self._motor_4_mms = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<h", self._motor_1_mms)
        buf += struct.pack("<h", self._motor_2_mms)
        buf += struct.pack("<h", self._motor_3_mms)
        buf += struct.pack("<h", self._motor_4_mms)
        return buf
    def get_motor_1_mms(self):
        return self._motor_1_mms
    def get_motor_2_mms(self):
        return self._motor_2_mms
    def get_motor_3_mms(self):
        return self._motor_3_mms
    def get_motor_4_mms(self):
        return self._motor_4_mms
    def get_all_data(self):
        data = []
        data.append((fields.motor_1_mms, self.get_motor_1_mms()))
        data.append((fields.motor_2_mms, self.get_motor_2_mms()))
        data.append((fields.motor_3_mms, self.get_motor_3_mms()))
        data.append((fields.motor_4_mms, self.get_motor_4_mms()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._motor_1_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        self._motor_2_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        self._motor_3_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        self._motor_4_mms = struct.unpack_from("<h", buf, index)[0]
        index += 2
        return
class set_buzzer_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.set_buzzer
        self._category = categories.none
        self._id = 8
        self._size = 1
        self._intensity = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_intensity(self, value):
        self._intensity = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._intensity)
        return buf
    def get_intensity(self):
        return self._intensity
    def get_all_data(self):
        data = []
        data.append((fields.intensity, self.get_intensity()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._intensity = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class set_led_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.set_led
        self._category = categories.none
        self._id = 9
        self._size = 3
        self._led_r = 0
        self._led_g = 0
        self._led_b = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_led_r(self, value):
        self._led_r = value
    def set_led_g(self, value):
        self._led_g = value
    def set_led_b(self, value):
        self._led_b = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._led_r)
        buf += struct.pack("<B", self._led_g)
        buf += struct.pack("<B", self._led_b)
        return buf
    def get_led_r(self):
        return self._led_r
    def get_led_g(self):
        return self._led_g
    def get_led_b(self):
        return self._led_b
    def get_all_data(self):
        data = []
        data.append((fields.led_r, self.get_led_r()))
        data.append((fields.led_g, self.get_led_g()))
        data.append((fields.led_b, self.get_led_b()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._led_r = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._led_g = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._led_b = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class poll_all_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.poll_all
        self._category = categories.none
        self._id = 16
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class poll_sensor_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.poll_sensor
        self._category = categories.none
        self._id = 17
        self._size = 1
        self._sensor_id = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_sensor_id(self, value):
        self._sensor_id = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._sensor_id)
        return buf
    def get_sensor_id(self):
        return self._sensor_id
    def get_all_data(self):
        data = []
        data.append((fields.sensor_id, self.get_sensor_id()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._sensor_id = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class start_stream_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.start_stream
        self._category = categories.none
        self._id = 18
        self._size = 8
        self._ip = 0
        self._port = 0
        self._rate_ms = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_ip(self, value):
        self._ip = value
    def set_port(self, value):
        self._port = value
    def set_rate_ms(self, value):
        self._rate_ms = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<L", self._ip)
        buf += struct.pack("<H", self._port)
        buf += struct.pack("<H", self._rate_ms)
        return buf
    def get_ip(self):
        return self._ip
    def get_port(self):
        return self._port
    def get_rate_ms(self):
        return self._rate_ms
    def get_all_data(self):
        data = []
        data.append((fields.ip, self.get_ip()))
        data.append((fields.port, self.get_port()))
        data.append((fields.rate_ms, self.get_rate_ms()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._ip = struct.unpack_from("<L", buf, index)[0]
        index += 4
        self._port = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._rate_ms = struct.unpack_from("<H", buf, index)[0]
        index += 2
        return
class stop_stream_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.stop_stream
        self._category = categories.none
        self._id = 19
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class set_node_id_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.set_node_id
        self._category = categories.none
        self._id = 20
        self._size = 1
        self._node_id = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_node_id(self, value):
        self._node_id = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._node_id)
        return buf
    def get_node_id(self):
        return self._node_id
    def get_all_data(self):
        data = []
        data.append((fields.node_id, self.get_node_id()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._node_id = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class wifi_config_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.wifi_config
        self._category = categories.none
        self._id = 21
        self._size = 96
        self._main_ssid = 0
        self._main_password = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_main_ssid(self, value):
        self._main_ssid = value
    def set_main_password(self, value):
        self._main_password = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<32s", self._main_ssid)
        buf += struct.pack("<64s", self._main_password)
        return buf
    def get_main_ssid(self):
        return self._main_ssid
    def get_main_password(self):
        return self._main_password
    def get_all_data(self):
        data = []
        data.append((fields.main_ssid, self.get_main_ssid()))
        data.append((fields.main_password, self.get_main_password()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._main_ssid = struct.unpack_from("<32s", buf, index)[0]
        index += 32
        self._main_password = struct.unpack_from("<64s", buf, index)[0]
        index += 64
        return
class ota_packet_from_control_to_durin:
    def __init__(self):
        self._sender = nodes.control
        self._receiver = nodes.durin
        self._message = messages.ota_packet
        self._category = categories.none
        self._id = 22
        self._size = 260
        self._frame_number = 0
        self._checksum = 0
        self._frame_length = 0
        self._data = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_frame_number(self, value):
        self._frame_number = value
    def set_checksum(self, value):
        self._checksum = value
    def set_frame_length(self, value):
        self._frame_length = value
    def set_data(self, value):
        self._data = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<H", self._frame_number)
        buf += struct.pack("<H", self._checksum)
        buf += struct.pack("<B", self._frame_length)
        buf += struct.pack("<255s", self._data)
        return buf
    def get_frame_number(self):
        return self._frame_number
    def get_checksum(self):
        return self._checksum
    def get_frame_length(self):
        return self._frame_length
    def get_data(self):
        return self._data
    def get_all_data(self):
        data = []
        data.append((fields.frame_number, self.get_frame_number()))
        data.append((fields.checksum, self.get_checksum()))
        data.append((fields.frame_length, self.get_frame_length()))
        data.append((fields.data, self.get_data()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._frame_number = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._checksum = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._frame_length = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._data = struct.unpack_from("<255s", buf, index)[0]
        index += 255
        return
class tof_12_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.tof_12
        self._category = categories.none
        self._id = 128
        self._size = 256
        self._tof_1 = 0
        self._tof_2 = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_tof_1(self, value):
        self._tof_1 = value
    def set_tof_2(self, value):
        self._tof_2 = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<128s", self._tof_1)
        buf += struct.pack("<128s", self._tof_2)
        return buf
    def get_tof_1(self):
        return [self._tof_1[i] + (self._tof_1[i + 1] << 8) for i in range(0, 128, 2)]
    def get_tof_2(self):
        return [self._tof_2[i] + (self._tof_2[i + 1] << 8) for i in range(0, 128, 2)]
    def get_all_data(self):
        data = []
        data.append((fields.tof_1, self.get_tof_1()))
        data.append((fields.tof_2, self.get_tof_2()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._tof_1 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        self._tof_2 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        return
class tof_34_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.tof_34
        self._category = categories.none
        self._id = 129
        self._size = 256
        self._tof_3 = 0
        self._tof_4 = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_tof_3(self, value):
        self._tof_3 = value
    def set_tof_4(self, value):
        self._tof_4 = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<128s", self._tof_3)
        buf += struct.pack("<128s", self._tof_4)
        return buf
    def get_tof_3(self):
        return [self._tof_3[i] + (self._tof_3[i + 1] << 8) for i in range(0, 128, 2)]
    def get_tof_4(self):
        return [self._tof_4[i] + (self._tof_4[i + 1] << 8) for i in range(0, 128, 2)]
    def get_all_data(self):
        data = []
        data.append((fields.tof_3, self.get_tof_3()))
        data.append((fields.tof_4, self.get_tof_4()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._tof_3 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        self._tof_4 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        return
class tof_56_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.tof_56
        self._category = categories.none
        self._id = 130
        self._size = 256
        self._tof_5 = 0
        self._tof_6 = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_tof_5(self, value):
        self._tof_5 = value
    def set_tof_6(self, value):
        self._tof_6 = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<128s", self._tof_5)
        buf += struct.pack("<128s", self._tof_6)
        return buf
    def get_tof_5(self):
        return [self._tof_5[i] + (self._tof_5[i + 1] << 8) for i in range(0, 128, 2)]
    def get_tof_6(self):
        return [self._tof_6[i] + (self._tof_6[i + 1] << 8) for i in range(0, 128, 2)]
    def get_all_data(self):
        data = []
        data.append((fields.tof_5, self.get_tof_5()))
        data.append((fields.tof_6, self.get_tof_6()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._tof_5 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        self._tof_6 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        return
class tof_78_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.tof_78
        self._category = categories.none
        self._id = 131
        self._size = 256
        self._tof_7 = 0
        self._tof_8 = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_tof_7(self, value):
        self._tof_7 = value
    def set_tof_8(self, value):
        self._tof_8 = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<128s", self._tof_7)
        buf += struct.pack("<128s", self._tof_8)
        return buf
    def get_tof_7(self):
        return [self._tof_7[i] + (self._tof_7[i + 1] << 8) for i in range(0, 128, 2)]
    def get_tof_8(self):
        return [self._tof_8[i] + (self._tof_8[i + 1] << 8) for i in range(0, 128, 2)]
    def get_all_data(self):
        data = []
        data.append((fields.tof_7, self.get_tof_7()))
        data.append((fields.tof_8, self.get_tof_8()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._tof_7 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        self._tof_8 = struct.unpack_from("<128s", buf, index)[0]
        index += 128
        return
class misc_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.misc
        self._category = categories.none
        self._id = 132
        self._size = 21
        self._charge_percent = 0
        self._battery_voltage_mv = 0
        self._ax = 0
        self._ay = 0
        self._az = 0
        self._gx = 0
        self._gy = 0
        self._gz = 0
        self._mx = 0
        self._my = 0
        self._mz = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_charge_percent(self, value):
        self._charge_percent = value
    def set_battery_voltage_mv(self, value):
        self._battery_voltage_mv = value
    def set_ax(self, value):
        self._ax = packedFloat_to_uint(value, -4, 4, 2)
    def set_ay(self, value):
        self._ay = packedFloat_to_uint(value, -4, 4, 2)
    def set_az(self, value):
        self._az = packedFloat_to_uint(value, -4, 4, 2)
    def set_gx(self, value):
        self._gx = packedFloat_to_uint(value, -4, 4, 2)
    def set_gy(self, value):
        self._gy = packedFloat_to_uint(value, -4, 4, 2)
    def set_gz(self, value):
        self._gz = packedFloat_to_uint(value, -4, 4, 2)
    def set_mx(self, value):
        self._mx = packedFloat_to_uint(value, -4, 4, 2)
    def set_my(self, value):
        self._my = packedFloat_to_uint(value, -4, 4, 2)
    def set_mz(self, value):
        self._mz = packedFloat_to_uint(value, -4, 4, 2)
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._charge_percent)
        buf += struct.pack("<H", self._battery_voltage_mv)
        buf += struct.pack("<H", self._ax)
        buf += struct.pack("<H", self._ay)
        buf += struct.pack("<H", self._az)
        buf += struct.pack("<H", self._gx)
        buf += struct.pack("<H", self._gy)
        buf += struct.pack("<H", self._gz)
        buf += struct.pack("<H", self._mx)
        buf += struct.pack("<H", self._my)
        buf += struct.pack("<H", self._mz)
        return buf
    def get_charge_percent(self):
        return self._charge_percent
    def get_battery_voltage_mv(self):
        return self._battery_voltage_mv
    def get_ax(self):
        return uint_to_packedFloat(self._ax, -4, 4, 2)
    def get_ay(self):
        return uint_to_packedFloat(self._ay, -4, 4, 2)
    def get_az(self):
        return uint_to_packedFloat(self._az, -4, 4, 2)
    def get_gx(self):
        return uint_to_packedFloat(self._gx, -4, 4, 2)
    def get_gy(self):
        return uint_to_packedFloat(self._gy, -4, 4, 2)
    def get_gz(self):
        return uint_to_packedFloat(self._gz, -4, 4, 2)
    def get_mx(self):
        return uint_to_packedFloat(self._mx, -4, 4, 2)
    def get_my(self):
        return uint_to_packedFloat(self._my, -4, 4, 2)
    def get_mz(self):
        return uint_to_packedFloat(self._mz, -4, 4, 2)
    def get_all_data(self):
        data = []
        data.append((fields.charge_percent, self.get_charge_percent()))
        data.append((fields.battery_voltage_mv, self.get_battery_voltage_mv()))
        data.append((fields.ax, self.get_ax()))
        data.append((fields.ay, self.get_ay()))
        data.append((fields.az, self.get_az()))
        data.append((fields.gx, self.get_gx()))
        data.append((fields.gy, self.get_gy()))
        data.append((fields.gz, self.get_gz()))
        data.append((fields.mx, self.get_mx()))
        data.append((fields.my, self.get_my()))
        data.append((fields.mz, self.get_mz()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._charge_percent = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._battery_voltage_mv = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._ax = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._ay = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._az = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._gx = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._gy = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._gz = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._mx = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._my = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._mz = struct.unpack_from("<H", buf, index)[0]
        index += 2
        return
class uwb_distance_from_durin_to_control:
    def __init__(self):
        self._sender = nodes.durin
        self._receiver = nodes.control
        self._message = messages.uwb_distance
        self._category = categories.none
        self._id = 133
        self._size = 1
        self._set_this_to_zero = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_set_this_to_zero(self, value):
        self._set_this_to_zero = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._set_this_to_zero)
        return buf
    def get_set_this_to_zero(self):
        return self._set_this_to_zero
    def get_all_data(self):
        data = []
        data.append((fields.set_this_to_zero, self.get_set_this_to_zero()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._set_this_to_zero = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
def id_to_message_class(id):
    if id == 0:
        receiver = acknowledge_from_durin_to_control()
        return receiver
    if id == 127:
        receiver = reject_from_durin_to_control()
        return receiver
    if id == 1:
        receiver = power_off_from_control_to_durin()
        return receiver
    if id == 2:
        receiver = move_rob_centric_from_control_to_durin()
        return receiver
    if id == 3:
        receiver = move_wheels_from_control_to_durin()
        return receiver
    if id == 8:
        receiver = set_buzzer_from_control_to_durin()
        return receiver
    if id == 9:
        receiver = set_led_from_control_to_durin()
        return receiver
    if id == 16:
        receiver = poll_all_from_control_to_durin()
        return receiver
    if id == 17:
        receiver = poll_sensor_from_control_to_durin()
        return receiver
    if id == 18:
        receiver = start_stream_from_control_to_durin()
        return receiver
    if id == 19:
        receiver = stop_stream_from_control_to_durin()
        return receiver
    if id == 20:
        receiver = set_node_id_from_control_to_durin()
        return receiver
    if id == 21:
        receiver = wifi_config_from_control_to_durin()
        return receiver
    if id == 22:
        receiver = ota_packet_from_control_to_durin()
        return receiver
    if id == 128:
        receiver = tof_12_from_durin_to_control()
        return receiver
    if id == 129:
        receiver = tof_34_from_durin_to_control()
        return receiver
    if id == 130:
        receiver = tof_56_from_durin_to_control()
        return receiver
    if id == 131:
        receiver = tof_78_from_durin_to_control()
        return receiver
    if id == 132:
        receiver = misc_from_durin_to_control()
        return receiver
    if id == 133:
        receiver = uwb_distance_from_durin_to_control()
        return receiver
def is_specifier(sender, name, field):
    if (messages.move_rob_centric == name and nodes.control == sender):
        if (fields.vel_x_mms == field):
            return False
        if (fields.vel_y_mms == field):
            return False
        if (fields.rot_degs == field):
            return False
    if (messages.move_wheels == name and nodes.control == sender):
        if (fields.motor_1_mms == field):
            return False
        if (fields.motor_2_mms == field):
            return False
        if (fields.motor_3_mms == field):
            return False
        if (fields.motor_4_mms == field):
            return False
    if (messages.set_buzzer == name and nodes.control == sender):
        if (fields.intensity == field):
            return False
    if (messages.set_led == name and nodes.control == sender):
        if (fields.led_r == field):
            return False
        if (fields.led_g == field):
            return False
        if (fields.led_b == field):
            return False
    if (messages.poll_sensor == name and nodes.control == sender):
        if (fields.sensor_id == field):
            return False
    if (messages.start_stream == name and nodes.control == sender):
        if (fields.ip == field):
            return False
        if (fields.port == field):
            return False
        if (fields.rate_ms == field):
            return False
    if (messages.set_node_id == name and nodes.control == sender):
        if (fields.node_id == field):
            return False
    if (messages.wifi_config == name and nodes.control == sender):
        if (fields.main_ssid == field):
            return False
        if (fields.main_password == field):
            return False
    if (messages.ota_packet == name and nodes.control == sender):
        if (fields.frame_number == field):
            return False
        if (fields.checksum == field):
            return False
        if (fields.frame_length == field):
            return False
        if (fields.data == field):
            return False
    if (messages.tof_12 == name and nodes.durin == sender):
        if (fields.tof_1 == field):
            return False
        if (fields.tof_2 == field):
            return False
    if (messages.tof_34 == name and nodes.durin == sender):
        if (fields.tof_3 == field):
            return False
        if (fields.tof_4 == field):
            return False
    if (messages.tof_56 == name and nodes.durin == sender):
        if (fields.tof_5 == field):
            return False
        if (fields.tof_6 == field):
            return False
    if (messages.tof_78 == name and nodes.durin == sender):
        if (fields.tof_7 == field):
            return False
        if (fields.tof_8 == field):
            return False
    if (messages.misc == name and nodes.durin == sender):
        if (fields.charge_percent == field):
            return False
        if (fields.battery_voltage_mv == field):
            return False
        if (fields.ax == field):
            return False
        if (fields.ay == field):
            return False
        if (fields.az == field):
            return False
        if (fields.gx == field):
            return False
        if (fields.gy == field):
            return False
        if (fields.gz == field):
            return False
        if (fields.mx == field):
            return False
        if (fields.my == field):
            return False
        if (fields.mz == field):
            return False
    if (messages.uwb_distance == name and nodes.durin == sender):
        if (fields.set_this_to_zero == field):
            return False
    return False
def is_extended_id(id):
    return
