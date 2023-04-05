import socket
import sys

import Utils

HOST = 'navio'#"127.0.0.1"  # The server's hostname or IP address

class OpAppSocket(Utils.BaseSocket):
    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, Utils.PORT))
        super().__init__( self.sock, buffSize=Utils.buffSize)

if __name__=='__main__':
    sys.path.append("..") # Adds higher directory to python modules path.
    ss = OpAppSocket()
    print("RECEIVED:", ss.getMessage())
    ss.sendRandomDict()
    ss.close()
