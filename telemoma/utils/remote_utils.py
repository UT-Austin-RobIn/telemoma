import pickle
import struct

# Implementation taken from https://twitter.com/francoisfleuret/status/1779260980398027261?t=7yPINrd59iyv5xGxBRxVKg&s=19
class SocketConnection:

    def __init__(self, socket):
        self.socket = socket
        self.socket.setblocking(1)
        self.buffer = b''
    
    def send(self, data):
        data = pickle.dumps(data)
        val = struct.pack('!i', len(data))
        self.socket.send(val)
        self.socket.sendall(data)

    def read(self, l):
        while len(self.buffer) < l:
            d = self.socket.recv(2**14)
            if d:
                self.buffer += d
        x = self.buffer[:l]
        self.buffer = self.buffer[l:]
        return x

    def read_data(self):
        l = struct.unpack('!i', self.read(4))[0]
        return pickle.loads(self.read(l))