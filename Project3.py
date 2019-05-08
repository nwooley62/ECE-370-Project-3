import socket
import sys
from ctypes import *
from getkey import getkey,keys
import time

velocity = 0
phi = 0

UDP_IP = "192.168.1.1"
UDP_PORT = 4242

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class cmdPacket(Structure):
    _fields_ = [("vel", c_double), ("theta", c_double), ("mode", c_int)]
    
class returnPacket(Structure):
    _fields_ = [("odo", d_double * 3), ("imu", c_double * 6), ("heading" c_double)]

while True:
    key = getkey()				#take key input
    
    if(key == keys.UP):
        velocity += 10
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 'keys.DOWN):
        velocity -= 10
        if(velocity < 0):
            velocity = 0
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == keys.LEFT):
        phi -= 15
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == keys.RIGHT):
        phi += 15
        sock.send(cmdPacket(velocity, phi, 1))
    elif(key == 'w'):
        velocity = 0
        phi = 0
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 'a'):
        velocity = 0
        phi = 270
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 's'):
        velocity = 0
        phi = 180
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == 'd'):
        velocity = 0 
        phi = 90
        sock.send(cmdPacket(velocity, phi, 2))
    elif(key == ' '):
        velocity = 0
        sock.send(cmdPacket(velocity, phi, 3))
    elif(key == key.SPACE):
	velocity = 0
        sock.send(cmdPacket(velocity, phi, 3))
    elif(key == 'r'):
        sock.send(cmdPacket(velocity,phi,4))
    else:
        print("Invalid Input")

    sock.send(cmdPacket(velocity, phi, 0))	#send a request for the info
    buffer = sock.recv(sizeof(returnPacket))
    tempPacket = returnPacket.from_buffer_copy(buffer)
    print "%d %d %d" % (tempPacket.odo[0], tempPacket.odo[1], tempPacket.odo[2])
    print "%d %d %d %d %d %d" % (tempPacket.imu[0], tempPacket.imu[1], tempPacket.imu[2], tempPacket.imu[3], tempPacket.imu[4], tempPacket.imu[5])
    print "%d" % (tempPacket.heading)
    time.sleep(0.1)
