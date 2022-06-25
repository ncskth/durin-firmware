import socket
from threading import Thread
from prot import *
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.animation

udp_local_ip = "0.0.0.0"
udp_local_port = 1336

tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_client.connect(("durin5.local", 1337))

udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_server.bind((udp_local_ip, udp_local_port))

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(("34.34.34.34", 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = "127.0.0.1"
    finally:
        s.close()
    return IP

global udp_buf
udp_buf = bytearray()
def receive(buf_len):
    global udp_buf
    while len(udp_buf) < buf_len:
        udp_buf += udp_server.recv(2048)
    ret = udp_buf[:buf_len]
    udp_buf = udp_buf[buf_len:]
    return ret

def send(msg):
    buf = bytearray([msg.get_id()]) + msg.build_buf()
    totalsent = 0
    while totalsent < len(buf):
        sent = tcp_client.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

# for the visualization
global tof_data
tof_data = [np.eye(8,8) * 200 for _ in range(8)]

def reader_thread():
    while True:
        msg_id = receive(1)[0] # first byte is always the ID
        message = id_to_message_class(msg_id) # defined in prot.py
        if message == None:
            print("invalid id")
            continue
        print(f"got {message.get_message()}")
        payload = receive(message.get_size()) # get the payload 
        message.parse_buf(payload)

        data = message.get_all_data() # returns all data for the class. there is also individual getters for the data
        for field in data:
            # for the visualization
            if "tof" in field[0].name:
                tof_i = int(field[0].name[4]) - 1
                distances = [0] * 64
                for i in range(64):
                    distance = field[1][i] & 0b0011111111111111
                    status = field[1][i] & 0b1100000000000000
                    distances[i] = distance
                tof_data[tof_i] = np.array(distances).reshape(8, 8)

stream_t = Thread(target = reader_thread)
stream_t.start()

# start streaming data
ip = [int(num) for num in get_ip().split(".")]
ip_int = (ip[3] << 24) + (ip[2] << 16) + (ip[1] << 8) + ip[0] 
msg = start_stream_from_control_to_durin()
msg.set_ip(ip_int)
msg.set_port(udp_local_port)
msg.set_rate_ms(67)
send(msg)

# visualization stuff
fig, axarr = plt.subplots(1, 8) 
objarr = [None] * 8
for i in range(8):
    objarr[i] = axarr[i].imshow(tof_data[i], interpolation='nearest', animated = True, vmin = 0, vmax = 500, cmap="copper_r")

def anim(n):
    for i in range(8):
        objarr[i].set_data(tof_data[i])
    return objarr
 
anim = matplotlib.animation.FuncAnimation(fig, anim, interval=67)
plt.show()