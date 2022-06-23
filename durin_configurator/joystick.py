from inputs import get_gamepad
import time
import socket
from prot import *

def norm(x):
    MIN = 21
    MAX = 232
    ABS = MAX - MIN
    MID = ABS / 2
    if x == 0:
        return x
    else:
        return ((x - MID - MIN) / ABS) * 1000

def send(msg):
    buf = bytearray([msg.get_id()]) + msg.build_buf()
    totalsent = 0
    while totalsent < len(buf):
        sent = tcp_client.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_client.connect(("durin4.local", 1337))

x = 0
y = 0
r = 0

while True:
    events = get_gamepad()
    for event in events:
        stick = event.code
        state = event.state
        if state > 0:
            if "ABS_X" in stick:
                x = state
            elif "ABS_Y" in stick:
                y = state
            elif "ABS_RX" in stick:
                r = state
    msg = move_rob_centric_from_control_to_durin()
    msg.set_vel_x_mms(-int(norm(x)))
    msg.set_vel_y_mms(-int(norm(y)))
    msg.set_rot_degs(-int(norm(r)))

    print(msg.get_all_data())
    send(msg)
    time.sleep(0.01)