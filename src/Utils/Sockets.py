import socket
import struct
import io
import json
import Utils.Common as Common
import random 


class BaseSocket:
    def __init__(self, conn, buffSize=4096) -> None:
        self.conn = conn
        self._recv_buffer = b""
        self.processingMsg = False
        self.buffSize = buffSize
        self.msgLen = -1

    def sendMessage(self, message) -> bool:
        json_bytes = self.__jsonEncode(message)
        hdr = struct.pack(">H", len(json_bytes))
        try:
            self.conn.sendall(hdr + json_bytes )
        except:
            Common.LogError("Error: Failed to send all data")
        return True

    def getMessage(self):
        self.__receive()
        self.__processHeader()
        data = self.__processContent()

        while data == None:
            # the entire message has not been received yet, so get another packet.
            self.__receive()
            data = self.__processContent()
            
        return data

    def close(self) -> None:
        self.conn.close()

    def sendRandomDict(self):
        #debug function
        randomdata = {
            "float from Drone": 3.1415926536,
            "random float from Drone": [random.random()]*int(random.random()*100),
            "string from Drone": "just a string",
            "list from Drone": [9999999,999999,99999]
        }
        print("SENDING:", randomdata)
        self.sendMessage( randomdata)

    def __receive(self):
        try:
            # Should be ready to read
            data = self.conn.recv(self.buffSize)
        except BlockingIOError:
            Common.LogError("Error: BlockingIOError")
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        else:
            if data:
                self._recv_buffer += data
            else:
                Common.LogError("Error: Peer closed")
                raise RuntimeError("Peer closed.")

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
        if len(self._recv_buffer) >= hdrlen:
            self.msgLen = struct.unpack(">H", self._recv_buffer[:hdrlen] )[0]
            self._recv_buffer = self._recv_buffer[hdrlen:]
            self.processingMsg = True
        
    def __processContent(self):
        if len(self._recv_buffer) < self.msgLen:
            # the receive buffer is incomplete, we need to wait for more messages
            #CommonUtils.LogDebug(str(len(self._recv_buffer))+',' +str(self.msgLen))
            return None

        data = self._recv_buffer[:self.msgLen]
        self._recv_buffer = self._recv_buffer[self.msgLen:]
        
        msg = self.__jsonDecode(data)
        Common.LogDebug("Received msg "+ str(msg))
        self.processingMsg = False
        return msg

class OperatorSocket(BaseSocket):
    def __init__(self) -> None:
        HOST = 'navio'#"127.0.0.1"  # The server's hostname or IP address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, Common.App_PORT))
        super().__init__( self.sock, buffSize=Common.App_buffSize)

class DroneSocket(BaseSocket) :
    def __init__(self) -> None:
        HOST = "" #empty to accept any client
        # the drone runs the server socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # SOCK_STREAM is the socket type for TCP
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # https://stackoverflow.com/questions/4465959/python-errno-98-address-already-in-use
        self.sock.bind((HOST, Common.App_PORT))
        self.sock.listen(5) # maximum of 5 connections
        conn, self.addr = self.sock.accept() # blocks execution, addr contains adress of the client
        super().__init__( conn, buffSize=Common.App_buffSize)


''' Extra Code
    def __GetAndProcessPacket(self):
        self.__receive()
        if not self.processingMsg:
            self.__processHeader()
        return self.__processContent()
'''