
import socket, threading


class SocketCallback(threading.Thread):

    def __init__(self, ip, port):
        super().__init__()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((ip, port))
        self.s.settimeout(1) # timeout after 1 second
        self.s.listen(1)
        self.callbacks = []
        self.running = True

    def run(self):
        pair = None
        while self.running:
            if pair:
                try:
                    sock, addr = pair
                    data = sock.recv(2048)
                    if not data:
                        pair = None     # connection closed
                        continue
                    for callback in self.callbacks: callback(sock, data)
                except Exception: pass
            else:
                try:
                    pair = self.s.accept()
                except: pass

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def close(self):
        self.running = False
        self.s.close()
