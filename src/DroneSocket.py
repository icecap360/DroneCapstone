import Utils.Sockets as Sockets
import socket
import Utils.Common as Common
import sys

HOST = "" #empty to accept any client

class DroneSocket(Sockets.BaseSocket) :
    def __init__(self) -> None:
        # the drone runs the server socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # SOCK_STREAM is the socket type for TCP
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # https://stackoverflow.com/questions/4465959/python-errno-98-address-already-in-use
        self.sock.bind((HOST, Common.App_PORT))
        self.sock.listen(5) # maximum of 5 connections
        conn, self.addr = self.sock.accept() # blocks execution, addr contains adress of the client
        super().__init__( conn, buffSize=Common.App_buffSize)


if __name__=='__main__':
    sys.path.append("..") # Adds higher directory to python modules path.
    ss = DroneSocket()
    ss.sendRandomDict()
    print("RECEIVED:", ss.getMessage())
    ss.close()


