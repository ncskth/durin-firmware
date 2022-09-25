import capnp
import socket
from threading import Thread

capnp.remove_import_hook()
schema = capnp.load('schema.capnp')

udp_local_ip = "0.0.0.0"
udp_local_port = 1336

tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_client.connect(("durin3.local", 1337))

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

def receive_tcp():
    header = tcp_client.recv(3)
    payload_size = header[1] + (header[2] << 8)
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

    print(buf)
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
            which = v.message.which()
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
            which = v.message.which()
            print("udp")
            print(which)
            print(v)
        

udp_t = Thread(target = udp_reader_thread)
udp_t.start()
tcp_t = Thread(target = tcp_reader_thread)
tcp_t.start()

msg = schema.DurinBase.new_message()
msg.message.init("getTofObservations")

msg.message.getTofObservations.ids = [0,1,2,3,4,5,6,7]

send_tcp(msg)

msg = schema.DurinBase.new_message()

msg.message.init("enableStreaming").destination.init("udpOnly")
ip = [int(x) for x in get_ip().split(".")]
msg.message.enableStreaming.destination.udpOnly.ip = ip
msg.message.enableStreaming.destination.udpOnly.port = 1336
send_tcp(msg)
