# Author: Ali 
# Date: December 2023
# Purpose: Implements message socket, used for communicating dictionaries.

import socket
import struct
import io
import json
import Utils.Common as Common
import random 
from time import sleep
import threading 
from abc import ABC, abstractmethod
import select

class MessageSocketParent(ABC):
    def __init__(self, conn, buffSize=4096) -> None:
        self.conn = conn
        self.connLock = threading.Lock()
        self.isConnected_b = Common.SharedVal(False)
        self.receivedMessages = Common.SharedQueue()
        self.messagesToSend = Common.SharedQueue()
        self.recvBuffer = b""
        self.processingMsg = False
        self.buffSize = buffSize
        self.msgLen = -1
        self.initialized = False

    def isInitialized(self):
        return self.initialized
    def init(self):
        self.connect()
        #self.__startReadThread()
        #self.__startSendThread()
        self.__startReadSendThread()
        self.initialized = True

    @abstractmethod
    def connect(self):
        self.isConnected_b.set(True)
        Common.LogMessage('Connect succeeded')
    def reconnect(self):
        self.isConnected_b.set(False)
        Common.LogError("Connection lost, reconnecting")
        self.connLock.acquire()
        self.conn.close()
        self.connLock.release()
        self.connect()

    def sendMessageSync(self, message: dict) -> bool:
        json_bytes = self.__jsonEncode(message)
        hdr = struct.pack(">H", len(json_bytes))
        try:
            self.connLock.acquire()
            self.conn.sendall(hdr + json_bytes )
            self.connLock.release()
            return True
        except:
            # connection to server lost
            if self.connLock.locked():
                self.connLock.release()
            return False

    def getMessage(self) -> bool:
        return self.receivedMessages.popLeft()
    def sendMessageAsync(self, data) -> bool:
        return self.messagesToSend.append(data)
    def isConnected(self) -> bool:
        return self.isConnected_b.get()
    def close(self) -> None:
        if self.isInitialized():
            self.__endReadSendThread()
            self.connLock.acquire()
            self.conn.close()
            self.connLock.release()

    def __startReadSendThread(self):
        self.threadReadSendEvent = threading.Event()
        self.threadReadSend = threading.Thread(target=self.__readSendMessagesInfLoop)
        self.threadReadSend.daemon = True
        self.threadReadSend.start()
    def __endReadSendThread(self):
        self.threadReadSendEvent.set()
    def __readSendMessagesInfLoop(self):
        while not self.threadReadSendEvent.is_set():
            readable, writable, exceptional = select.select([self.conn], [self.conn], [self.conn])
            if len(exceptional) > 0:
                self.reconnect()

            if len(readable) > 0:
                if not self.__receive():
                    self.reconnect()
                while self.recvBuffer:
                    self.__processHeader()
                    oneMessage = self.__processContent()
                    while oneMessage == None:
                        # the entire message has not been received yet, so get another packet.
                        if not self.__receive():
                            self.reconnect()
                        oneMessage = self.__processContent()
                    self.receivedMessages.append(oneMessage)

            if len(writable) > 0: 
                oneMessage = self.messagesToSend.popLeft()
                if oneMessage:
                    ret = self.sendMessageSync(oneMessage)
                    while ret != True:
                        self.reconnect()
                        ret = self.sendMessageSync(oneMessage)

    def __receive(self) -> bool:    
        try:
            # Should be ready to read
            self.connLock.acquire()
            data = self.conn.recv(self.buffSize)
            self.connLock.release()
        except:
            if self.connLock.locked():
                self.connLock.release()
            return False
        else:
            if data:
                self.recvBuffer += data
                return True
            else:
                return False 

    def __jsonEncode(self, obj):
        return json.dumps(obj, ensure_ascii=False).encode("utf-8")

    def __jsonDecode(self, json_bytes):
        tiow = io.TextIOWrapper(
            io.BytesIO(json_bytes), encoding="utf-8", newline=""
        )
        obj = json.load(tiow)
        tiow.close()
        return obj

    def __processHeader(self):
        hdrlen = 2
        if len(self.recvBuffer) >= hdrlen:
            self.msgLen = struct.unpack(">H", self.recvBuffer[:hdrlen] )[0]
            self.recvBuffer = self.recvBuffer[hdrlen:]
            self.processingMsg = True
        
    def __processContent(self):
        if len(self.recvBuffer) < self.msgLen:
            # the receive buffer is incomplete, we need to wait for more messages
            #CommonUtils.LogDebug(str(len(self._recv_buffer))+',' +str(self.msgLen))
            return None
        data = self.recvBuffer[:self.msgLen]
        self.recvBuffer = self.recvBuffer[self.msgLen:]
        msg = self.__jsonDecode(data)
        Common.LogDebug("Received msg "+ str(msg))
        self.processingMsg = False
        return msg
    
    def sendRandomDict(self):
        # testing function
        randomdata = {
            "float from Drone": 3.1415926536,
            "random float from Drone": [random.random()]*int(random.random()*100),
            "string from Drone": "just a string",
            "list from Drone": [9999999,999999,99999]
        }
        print("SENDING:", randomdata)
        self.sendMessage( randomdata)


class MessageSocket(MessageSocketParent) :
    def __init__(self, type, HOST=None, PORT=3002) -> None:
        self.type = type
        self.HOST = HOST
        self.PORT = PORT
        self.buffSize=4096
        super().__init__(self)
        # the drone runs the server socket, while the operator runs the 
    def connect(self):
        if self.type == "DRONE":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # SOCK_STREAM is the socket type for TCP
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # https://stackoverflow.com/questions/4465959/python-errno-98-address-already-in-use
            HOST = "" # empty to accept any client
            self.sock.bind((HOST, self.PORT))
            self.sock.listen(500) # maximum of 500 connections, so I think we can lose and regain connection 500 times
            Common.LogMessage('Waiting for connections')
            conn = None
            while True:
                readable, _, _ = select.select([self.sock], [], [])
                sleep(0.5)
                if len(readable) >0 and readable[0] is self.sock:
                    conn, self.addr = self.sock.accept()
                    break
            super().__init__( conn, buffSize=self.buffSize)
            super().connect()
        elif self.type == "OPERATOR":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKfiET, socket.SO_REUSEADDR, 1)
            ret = self.sock.connect_ex((self.HOST, self.PORT))
            while ret != 0:
                Common.LogMessage('Connect failed, waiting 3 sec')
                sleep(3)
                ret = self.sock.connect_ex((self.HOST, self.PORT))
            super().__init__( self.sock, buffSize=self.buffSize)
            super().connect()
        else:
            Common.LogError("Invalid Socket Type:"+str(self.type))

