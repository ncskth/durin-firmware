import threading
import serial
import sys
import socket
import capnp
import traceback
import time

capnp.remove_import_hook()
schema = capnp.load('schema.capnp')

UART = 0
TCP = 1
UDP = 2

def encode_msg(msg):
    payload = msg.to_bytes_packed()
    buf = bytearray()
    buf += b'*'
    buf.append(len(payload) & 0xff)
    buf.append((len(payload) >> 8) & 0xff)
    buf += payload
    checksum = 0
    for v in buf:
        checksum ^= v
    buf += bytearray([checksum])
    return buf



uart_tx_queue = []
tcp_tx_queue = []
udp_tx_queue = []

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.bind(("0.0.0.0", 1337))
tcp_user_socket = None

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if len(sys.argv) != 2:
    print("please provide a UART port")

ser = serial.Serial(sys.argv[1], 2e6, rtscts=True)
ser.read_all()

def tcp_rx_thread():
    def tcp_receive(msg_len, s):
        chunks = []
        bytes_recd = 0
        while bytes_recd < msg_len:
            chunk = s.recv(min(msg_len - bytes_recd, 2048))
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
            buf = b''.join(chunks)
            return buf

    # client loop
    while True:
        try:
            tcp_socket.listen(1)
            global tcp_user_socket
            (tcp_user_socket, address) = tcp_socket.accept()
            # protocol loop
            while True:
                buf = bytes()
                header = tcp_receive(1, tcp_user_socket)
                if header != b"*":
                    print("tcp invalid header", header)
                    continue
                buf += header
                header2 = tcp_receive(2, tcp_user_socket)
                buf += header2
                payload_len = header2[0] + (header2[1] << 8)
                meta = 0xf000 & payload_len
                payload_len = 0x0fff & payload_len
                payload = tcp_receive(payload_len, tcp_user_socket)
                buf += payload
                checksum = tcp_receive(1, tcp_user_socket)[0]
                buf += bytes([checksum])
                calculated_checksum = ord("*") ^ header2[0] ^ header2[1]
                for v in payload:
                    calculated_checksum ^= v
                if calculated_checksum != checksum:
                    print(f"tcp invalid checksum. calculated: {calculated_checksum} received: {checksum}")
                    continue
                if (len(payload) != payload_len):
                    print("tcp got the wrong length", len(payload), payload_len)
                    continue
                msg = schema.DurinBase.from_bytes_packed(payload)
                # print("tcp got message", msg.which())
                if msg.which() == "enableStreaming":
                    ip = None
                    port = None
                    if msg.enableStreaming.destination.which() == "uartAndUdp":
                        ip = msg.enableStreaming.destination.uartAndUdp.ip
                        port = msg.enableStreaming.destination.uartAndUdp.port
                    elif msg.enableStreaming.destination.which() == "udpOnly":
                        ip = msg.enableStreaming.destination.udpOnly.ip
                        port = msg.enableStreaming.destination.udpOnly.port
                    else:
                        print("got stream over uart message")
                        continue
                    ip = map(str, ip)
                    ip_str = ".".join(ip)
                    print("enable streaming", ip_str, port)
                    udp_socket.connect((ip_str, port))
                    replacement = msg
                    replacement = schema.DurinBase.new_message()
                    replacement.init("enableStreaming")
                    replacement.enableStreaming.destination.uartOnly = None
                    buf = encode_msg(replacement)
                    uart_tx_queue.append(buf)
                else:
                    uart_tx_queue.append(buf)
        except Exception:
            traceback.print_exc()

def uart_rx_thread():
    while True:
        buf = bytes()
        header = ser.read(1)
        if header != b"*":
            print("uart invalid header", header)
            continue
        buf += header
        header2 = ser.read(2)
        buf += header2
        payload_len = header2[0] + (header2[1] << 8)
        meta = 0xf000 & payload_len
        payload_len = 0x0fff & payload_len
        payload = ser.read(payload_len)
        buf += payload
        checksum = ser.read(1)[0]
        buf += bytes([checksum])
        calculated_checksum = ord("*") ^ header2[0] ^ header2[1]
        for v in payload:
            calculated_checksum ^= v
        if calculated_checksum != checksum:
            print(f"uart invalid checksum. calculated: {calculated_checksum} received: {checksum}")
        if (len(payload) != payload_len):
            print("uart got the wrong length", len(payload), payload_len)
        msg = schema.DurinBase.from_bytes_packed(payload)
        # print("uart got message", msg.which())
        if meta:
            udp_tx_queue.append(buf)
        else:
            tcp_tx_queue.append(buf)

def udp_tx_thread():
    def udp_send(buf, s):
        totalsent = 0
        while totalsent < len(buf):
            sent = s.send(buf[totalsent:])
            if sent == 0:
                break
                # raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent
    while True:
        try:
            if len(udp_tx_queue) > 0:
                # print("udp tx")
                udp_send(udp_tx_queue.pop(0), udp_socket)
            else:
                time.sleep(0.01)
        except Exception:
            traceback.print_exc()

def tcp_tx_thread():
    def tcp_send(buf, s):
        totalsent = 0
        while totalsent < len(buf):
            sent = s.send(buf[totalsent:])
            if sent == 0:
                break
                # raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent
    while True:
        try:
            if len(tcp_tx_queue) > 0:
                # print("tcp tx")
                tcp_send(tcp_tx_queue.pop(0), tcp_user_socket)
            else:
                time.sleep(0.01)
        except Exception:
            traceback.print_exc()

def uart_tx_thread():
    while True:
        if len(uart_tx_queue) > 0:
            # print("uart tx")
            buf = uart_tx_queue.pop(0)
            ser.write(buf)
        else:
            time.sleep(0.01)

tcp_rx_t = threading.Thread(target=tcp_rx_thread)
tcp_tx_t = threading.Thread(target=tcp_tx_thread)
udp_tx_t = threading.Thread(target=udp_tx_thread)
uart_rx_t = threading.Thread(target=uart_rx_thread)
uart_tx_t = threading.Thread(target=uart_tx_thread)


tcp_rx_t.start()
tcp_tx_t.start()
udp_tx_t.start()
uart_rx_t.start()
uart_tx_t.start()