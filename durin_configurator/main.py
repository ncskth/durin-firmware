import argparse
import os
import time
import socket
import sys
from prot import *

def crc16(data : bytearray, offset , length):
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    crc = 0
    for i in range(0, length):
        crc ^= data[offset + i]
        for j in range(0,8):
            if (crc & 1) > 0:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc = crc >> 1
    return crc


def send(msg):
    buf = bytearray([msg.get_id()]) + msg.build_buf()
    totalsent = 0
    while totalsent < len(buf):
        sent = s.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

def receive(msg_len):
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

def wait_ack():
    buf = receive(1)
    if buf[0] == acknowledge_from_durin_to_control().get_id():
        return True
    else:
        return False

parser = argparse.ArgumentParser(description='durin configurator')
parser.add_argument('ip', type=str,
                    help='ip')

parser.add_argument('--id', type=int,
                    help='set the node id')

parser.add_argument('--wifi', nargs = 2, type=str,
                    help='--wifi [ssid] [password]')


parser.add_argument('--firmware', type=str,
                    help="upload a firmware file")

parser.add_argument('--verify', action='store_true',
                    help="verify and apply the new firmware")

args = parser.parse_args()

print("connecting...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((args.ip, 1337))
print("connected")

if args.wifi:
    print("updating wifi")
    msg = wifi_config_from_control_to_durin()
    msg.set_main_ssid(bytes(args.wifi[0], "ASCII"))
    msg.set_main_password(bytes(args.wifi[1], "ASCII"))
    send(msg)
    if not wait_ack():
        print("failed to set wifi")

if args.id:
    print("updating node id")
    msg = set_node_id_from_control_to_durin()
    msg.set_node_id(args.id)
    send(msg)
    if not wait_ack():
        print("failed to set node id")

success = True
if args.firmware:
    print("updating firmware")
    f = open(args.firmware, "rb")
    size = os.path.getsize(args.firmware)

    frame_number = 0
    while True:
        msg = ota_packet_from_control_to_durin()
        buf = f.read(255)
        crc = crc16(buf, 0, len(buf))
        msg.set_checksum(crc)
        msg.set_data(buf)
        msg.set_frame_length(len(buf))
        print(len(buf))
        msg.set_frame_number(frame_number)
        frame_number += 1
        send(msg)
        print(f"sent ota packet. {round(frame_number * 255 / size * 100, 2)}% done")
        if len(buf) < 255:
            success = True
            break
        if not wait_ack():
            success = False
            print("failed to send an ota packet\naborting")
            break

if success and args.verify:
    if args.firmware:
        print("upload done, power on during again.")
        time.sleep(5)
        while True:
            try:
                s.close()
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                print("trying to connect...")
                s.connect((args.ip, 1337))
                s.settimeout(None)
                break
            except:
                pass
        print("connected")

    print("sending an all zeros ota packet to confirm a successful update")
    msg = ota_packet_from_control_to_durin()
    msg.set_checksum(0)
    msg.set_data(bytes([0] * 255))
    msg.set_frame_length(0)
    msg.set_frame_number(0)
    send(msg)
    if not wait_ack():
        print("could not confirm the update and it will be rolled back when durin shuts down. confirm manually if you change the network configuration")
    else:
        print("updated!")


