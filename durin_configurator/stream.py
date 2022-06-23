import socket
from threading import Thread
from prot import *

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

def receive(socket, buf_len):
    chunks = []
    bytes_recd = 0
    while bytes_recd < buf_len:
        chunk = socket.recv(min(buf_len - bytes_recd, 1024))
        if chunk == b'':
            raise RuntimeError("socket connection broken")
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)
    buf = b''.join(chunks)
    return buf

def send(msg):
    buf = bytearray([msg.get_id()]) + msg.build_buf()
    totalsent = 0
    while totalsent < len(buf):
        sent = tcp_client.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

def reader_thread():
    while True:
        msg_id = receive(udp_server, 1)[0]
        message = id_to_message_class(msg_id)
        if message == None:
            print("invalid id")
            continue
        print(f"got {message.get_message()}")
        payload = receive(udp_server, message.get_size())
        message.parse_buf(payload)

        data = message.get_all_data()
        for field in data:
            pass
            #print(f"\t{field[0]}: {field[1]}")

t = Thread(target = reader_thread)
t.start()

ip = [int(num) for num in get_ip().split(".")]
ip_int = (ip[3] << 24) + (ip[2] << 16) + (ip[1] << 8) + ip[0]
print(ip_int)
msg = start_stream_from_control_to_durin()
msg.set_ip(ip_int)
msg.set_port(udp_local_port)
msg.set_rate_ms(5000)

send(msg)