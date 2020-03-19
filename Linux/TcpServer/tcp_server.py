#!/usr/bin/env python
# ======================================================
# Copyright (C) 2020 repa1030
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================
''' 
TCP Server for receiving images from uniy on linux
Requires openCV and numpy for python
'''
import socket
import numpy as np
import cv2
import time

class TcpServer:

    def __init__(self, tcp_ip, tcp_port, buffer_size, time_out):
        self.ip = tcp_ip
        self.port = tcp_port
        self.buf_size = buffer_size
        self.conn = None
        self.addr = None
        self.tcp_time_out = time_out

    def connectToClient(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.ip, self.port))
        s.listen(1)
        self.conn, self.addr = s.accept()
        self.conn.settimeout(self.tcp_time_out)

    def closeConnection(self):
        self.conn.close()

    def reconnect(self):
        self.conn.close()
        print ('Reconnecting to Client...')
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.ip, self.port))
        s.listen(1)
        self.conn, self.addr = s.accept()
        self.conn.settimeout(self.tcp_time_out)
        print ('Connection to Client is established.')

    def receiveMessage(self):
        msg = ''
        try:
            # While there is content in buffer
            while True:
                data = self.conn.recv(self.buf_size)
                msg = msg + data
                if len(data) < self.buf_size: break
                data = None
            # Check if buffer is not empty
        except socket.timeout:
            return (None, None, False)
        img = None
        if len(msg) > 0:
            # Decoding jpg image to array and read current velocity
            msg = np.fromstring(msg, np.uint8)
            velo = msg[-2]
            img = cv2.imdecode(msg[:-2], -1)
        if type(img) is np.ndarray:
            return (img, velo, True)
        return (None, None, True)

    def sendMessage(self, m_x, m_y, m_z):
        msg = 'START@' + str(m_x) + ';' + str(m_y) + ';' + str(m_z) + '@END'
        try:
            self.conn.send(msg)
            return True
        except:
            return False
