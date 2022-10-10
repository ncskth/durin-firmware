import argparse
import os
import time
import socket
import sys
import serial
import capnp
import random
import math

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

parser.add_argument('--move', nargs=3, type=int)

parser.add_argument('--dict', type=str)

parser.add_argument('--uart-stream', action="store_true")
parser.add_argument('--spam', action="store_true")

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
    meta = 0xf000 & payload_len
    payload_len = 0x0fff & payload_len
    payload = receive_raw(payload_len)
    return schema.DurinBase.from_bytes(payload)

def wait_ack(quiet = False):
    while True:
        msg = receive_msg()
        base = next(msg.gen)
        if base.which() == "acknowledge":
            if not quiet:
                print("acknowledged")
            return True
        if base.which() == "reject":
            if not quiet:
                print("rejected")
            return False
        print("got another message. sus", base.which())
print("connecting...")
s = None
ser = None
if args.type == "wifi":
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.address, 1337))

if args.type == "uart":
    ser = serial.Serial(args.address, 2e6)
    ser.read_all()

print("connected")

# generic way to send messages
# for example this is a valid argument  "{'setLed': {'ledR': 255, 'ledG':255, 'ledB':255}}"
if args.dict:
    print("sending generic message")
    msg = schema.DurinBase.new_message(**eval(args.dict))
    send_msg(msg)
    response = receive_msg()
    print(next(response.gen))

if args.set_led:
    print("setting the LED")
    msg = schema.DurinBase.new_message()
    msg.init("setLed")
    msg.setLed.ledR = args.set_led[0]
    msg.setLed.ledG = args.set_led[1]
    msg.setLed.ledB = args.set_led[2]
    send_msg(msg)
    wait_ack()

if args.move:
    print("setting the velocity")
    msg = schema.DurinBase.new_message()
    msg.init("setRobotVelocity")
    msg.setRobotVelocity.velocityXMms = args.move[0]
    msg.setRobotVelocity.velocityYMms = args.move[1]
    msg.setRobotVelocity.rotationDegs = args.move[2]
    send_msg(msg)
    wait_ack()

if args.wifi:
    print("updating wifi")
    msg = schema.DurinBase.new_message()
    msg.init("setWifiConfig")
    msg.setWifiConfig.ssid = args.wifi[0]
    msg.setWifiConfig.password = args.wifi[1]

    send_msg(msg)
    wait_ack()

if args.id:
    print("updating node id")
    msg = schema.DurinBase.new_message()
    msg.init("setNodeId")
    msg.setNodeId.nodeId = args.id
    send_msg(msg)
    wait_ack()
    
if args.read_logs:
    print("enabling logging")
    msg = schema.DurinBase.new_message()
    msg.init("enableLogging")
    if args.type == "wifi":
        msg.enableLogging.tcp = None
    if args.type == "uart":
        msg.enableLogging.uart = None

    send_msg(msg)
    wait_ack()
    print("log output")
    while True:
        log = receive_msg()
        base = next(log.gen)
        if base.which() == "textLogging":
            print(base.textLogging.log, end="")
        else:
            print(base)

if args.uart_stream:
    msg = schema.DurinBase.new_message()
    msg.init("enableStreaming")
    msg.enableStreaming.destination.uartOnly = None

    send_msg(msg)
    wait_ack()
    print("stream output")
    while True:
        log = receive_msg()
        base = next(log.gen)
        print(base)


success = True
if args.firmware:
    print("updating firmware")
    f = open(args.firmware, "rb")
    size = os.path.getsize(args.firmware)
    begin_msg = schema.DurinBase.new_message()
    begin_msg.init("otaUpdateBegin")
    send_msg(begin_msg)
    if not wait_ack(True):
        print("Error while starting the OTA update")
        sys.exit()
    print("OTA started")
    sent = 0
    while True:
        data_msg = schema.DurinBase.new_message()
        data_msg.init("otaUpdate")
        buf = f.read(1024)
        data_msg.otaUpdate.data = buf
        sent += len(buf)
        send_msg(data_msg)
        print(f"sent ota packet. {round(sent / size * 100, 2)}% done")
        if not wait_ack(True):
            success = False
            print("failed to send an ota packet\naborting")
            sys.exit()
        if len(buf) < 255:
            success = True
            print("upload done, commiting")
            break

    commit_msg = schema.DurinBase.new_message()
    commit_msg.init("otaUpdateCommit")
    send_msg(commit_msg)
    # durin powers off before it can send a response
    # if wait_ack(True):
    #     print("flashing done")
    # else:
    #     print("could not commit firmware udpate")
    #     success = False

if success and args.verify:
    if args.firmware:
        print("upload done, power on durin again.")
        time.sleep(5)
        while True:
            try:
                s.close()
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                print("trying to connect...")
                s.connect((args.address, 1337))
                s.settimeout(None)
                break
            except KeyboardInterrupt:
                sys.exit()
            except Exception:
                pass
        print("connected")

    print("confirming the update")
    verify_msg = schema.DurinBase.new_message()
    verify_msg.init("otaUpdateCommit")
    send_msg(verify_msg)
    if not wait_ack():
        print("could not confirm the update and it will be rolled back when durin shuts down")
    else:
        print("updated!")

if args.spam:
    print("IT'S A PARTY")
    
    r = 0
    g = 0
    b = 0
    while True:
        r += math.floor(random.random() * 3)
        g += math.floor(random.random() * 3)
        b += math.floor(random.random() * 3)

        r = r % 64
        g = g % 64
        b = b % 64
        msg = schema.DurinBase.new_message()
        msg.init("setLed")
        msg.setLed.ledR = r
        msg.setLed.ledG = g
        msg.setLed.ledB = b
        send_msg(msg)
        wait_ack()