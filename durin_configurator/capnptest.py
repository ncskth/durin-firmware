import capnp
import socket
from threading import Thread
import time
import random
from inputs import get_gamepad
import math
import sys

capnp.remove_import_hook()
schema = capnp.load('schema.capnp')

udp_local_ip = "0.0.0.0"
udp_local_port = int(sys.argv[2])

tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_client.connect((sys.argv[1], 1337))
udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_server.bind((udp_local_ip, udp_local_port))


print("connected")

def norm(x):
    MIN = 15
    MAX = 232
    ABS = MAX - MIN
    MID = ABS / 2
    if x == 0:
        return x
    else:
        return ((x - MID - MIN) / ABS) * 1000

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

def receive_tcp():
    header = tcp_client.recv(3)
    payload_size = header[1] + ((header[2] & 0x0f) << 8)
    meta = header[2] &0xf000
    print(meta)
    payload = tcp_client.recv(payload_size)
    msg = schema.DurinBase.from_bytes(payload)
    return msg

def send_tcp(msg):
    payload = msg.to_bytes()
    buf = bytearray()
    buf += b"\n"
    buf.append(len(payload) & 0xff)
    buf.append((len(payload) >> 8) & 0xff)
    buf += payload

    # print(buf)
    totalsent = 0
    while totalsent < len(buf):
        sent = tcp_client.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

def tcp_reader_thread():
    while True:
        msg = receive_tcp()
        for v in msg.gen:
            which = v.which()
            print("tcp")
            print(which)
            print(v)

def udp_reader_thread():
    while True:
        buf = udp_server.recv(2048)
        if buf[0] != b"\n"[0]:
            print("udp: invalid header")
        payload_len = buf[1] + (buf[2] << 8)
        
        if(len(buf) != payload_len + 3):
            print("invalid len")
        msg = schema.DurinBase.from_bytes(buf[3:])
        for v in msg.gen:
            which = v.which()
            print("udp")
            print(which)
            print(v)

udp_t = Thread(target = udp_reader_thread)
udp_t.start()
tcp_t = Thread(target = tcp_reader_thread)
# tcp_t.start()

msg = schema.DurinBase.new_message()
msg.init("getTofObservations")
msg.getTofObservations.ids = [0,1,2,3,4,5,6,7]
send_tcp(msg)

msg = schema.DurinBase.new_message()
msg.init("enableStreaming").destination.init("udpOnly")
ip = [int(x) for x in get_ip().split(".")]
msg.enableStreaming.destination.udpOnly.ip = ip
msg.enableStreaming.destination.udpOnly.port = udp_local_port
send_tcp(msg)

msg = schema.DurinBase.new_message()
msg.init("setTofStreamPeriod").periodMs = schema.streamPeriodMin
send_tcp(msg)


# time.sleep(5)
# msg = schema.DurinBase.new_message()
# msg.init("disableStreaming")
# send_tcp(msg)

r = 0
g = 0
b = 0
while True:
    r += 1
    g += 2
    b += 3

    r = r % 255
    g = g % 255
    b = b % 255
    msg = schema.DurinBase.new_message()
    msg.init("setLed")
    msg.setLed.ledR = r
    msg.setLed.ledG = g
    msg.setLed.ledB = b
    send_tcp(msg)
    time.sleep(0.01)

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

    msg = schema.DurinBase.new_message()
    msg.init("setRobotVelocity")
    if math.sqrt(norm(x)**2 + norm(y)**2) > 100:
        msg.setRobotVelocity.velocityXMms = int(norm(x))
        msg.setRobotVelocity.velocityYMms = int(norm(y))
    else:
        msg.setRobotVelocity.velocityXMms = 0
        msg.setRobotVelocity.velocityYMms = 0
        msg.setRobotVelocity.rotationDegs = 0

    if (abs(norm(r)) > 50):
        msg.setRobotVelocity.rotationDegs = int(norm(r))
    else:
        msg.setRobotVelocity.rotationDegs = 0
        
    send_tcp(msg)
    time.sleep(0.01)
