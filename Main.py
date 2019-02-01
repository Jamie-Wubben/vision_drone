import argparse
import time

from Camera import Camera
from SocketCallback import SocketCallback


class listener:
    def __init__(self, working_mode, ip, port):
        self.working_mode = working_mode
        self.ip = ip
        # TODO check port if multiple drones are connected
        self.port = port
        command_socket = SocketCallback(self.ip, self.port)
        command_socket.add_callback(self.handle_command)
        command_socket.start()
        self.cam = Camera(working_mode)

    def handle_command(self, sock, data):
        # decode the data and remove \r\n
        # TODO try to use getattribute to clean the code.
        # getattr(Camera, "foo")()
        command = data.decode().rstrip()
        print(command)
        success = False
        if command == 'start_camera':
            self.cam.start()
            success = True
        elif command == 'stop_camera':
            self.cam.stop_camera()
            success = True
        elif command == 'find_target':
            self.cam.find_target()
            success = True
        else:
            success = False

        if success:
            sock.send("ACK\n".encode())
        else:
            sock.send("ACK\n".encode())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-w", "--working_mode", help="choice working mode <pc or pi>")
    parser.add_argument("-i", "--ip", help="ip address of host for socket with java")
    parser.add_argument("-p", "--port", help="port for socket with java")
    args = parser.parse_args()

    working_mode = "pc" if not args.working_mode else args.working_mode
    port = 5761 if not args.port else int(args.port)
    ip = "localhost" if not args.ip else args.ip

    print("start python listener")
    print("working mode: " + working_mode)
    t = listener(working_mode, ip, port)
    while True:
        time.sleep(100)
