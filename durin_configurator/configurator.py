import argparse
import os
import time
import socket
import sys
import serial
import capnp

capnp.remove_import_hook()
schema = capnp.load('schema.capnp')

parser = argparse.ArgumentParser(description='durin configurator')

parser.add_argument('type', choices=['uart', 'wifi'])

parser.add_argument('address', type=str, help = "[serial path]/[ip_address:port]")

parser.add_argument('--id', type=int,
                    help='set the node id')

parser.add_argument('--wifi', nargs = 2, type=str,
                    help='--wifi [ssid] [password]')

parser.add_argument('--firmware', type=str,
                    help="upload a firmware file")

parser.add_argument('--verify', action='store_true',
                    help="verify and apply the new firmware")

parser.add_argument('--read-logs', action='store_true')

parser.add_argument('--set-led', nargs=3, type=int)

parser.add_argument('--dict', type=dict)


args = parser.parse_args()

def send_msg(msg):
    payload = msg.to_bytes()
    buf = bytearray()
    buf += b"\n"
    buf.append(len(payload) & 0xff)
    buf.append((len(payload) >> 8) & 0xff)
    buf += payload

    if args.type == "wifi":
        totalsent = 0
        while totalsent < len(buf):
            sent = s.send(buf[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    if args.type == "uart":
        ser.write(buf)

def receive_raw(msg_len):
    if args.type == "wifi":
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
    if args.type == "uart":
        return ser.read(msg_len)

def receive_msg():
    while True:
        header = receive_raw(1)
        if header == b"\n":
            break
        print("invalid header", header)
        
    header2 = receive_raw(2)
    payload_len = header2[0] + (header2[1] << 8)
    payload = receive_raw(payload_len)
    return schema.DurinBase.from_bytes(payload)

def wait_ack():
    while True:
        msg = receive_msg()
        base = next(msg.gen)
        print(base)
        if base.message.which() == "acknowledge":
            print("acknowledged")
            return True
        if base.message.which() == "reject":
            print("rejected")
            return False

print("connecting...")
s = None
ser = None
if args.type == "wifi":
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.address, 1337))

if args.type == "uart":
    ser = serial.Serial(args.address, 2e6)
print("connected")

if args.set_led:
    msg = schema.DurinBase.new_message()
    msg.message.init("setLed")
    msg.message.setLed.ledR = args.set_led[0]
    msg.message.setLed.ledG = args.set_led[1]
    msg.message.setLed.ledB = args.set_led[2]
    send_msg(msg)
    wait_ack()

if args.wifi:
    print("updating wifi")
    msg = schema.DurinBase.new_message()
    msg.message.init("setWifiConfig")
    msg.message.setWifiConfig.ssid = args.wifi[0]
    msg.message.setWifiConfig.password = args.wifi[1]

    send_msg(msg)
    if not wait_ack():
        print("failed to set wifi")

if args.id:
    print("updating node id")
    msg = set_node_id_from_control_to_durin()
    msg.set_node_id(args.id)
    send(msg)
    if not wait_ack():
        print("failed to set node id")
    
if args.read_logs:
    msg = schema.DurinBase.new_message()
    msg.message.init("enableLogging")
    msg.message.enableLogging.enabled = True
    send_msg(msg)
    wait_ack()
    while True:
        log = receive_msg()
        base = next(log.gen)
        if base.message.which() == "textLogging":
            print(base.message.textLogging.log, end="")
        else:
            print(base)

if args.dict:
    msg_dict = {message: args.dict}
    print(msg_dict)
    # msg = schema.DurinBase.new_message(...msg_dict)
    send_msg(msg)

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
            except KeyboardInterrupt:
                sys.exit()
            except Exception:
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
        print("could not confirm the update and it will be rolled back when durin shuts down")
    else:
        print("updated!")