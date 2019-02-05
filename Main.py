import argparse
import time, sys, signal

from Camera import Camera
from SocketCallback import SocketCallback

main_running = True

class listener:
    def __init__(self, working_mode, ip, port):
        self.working_mode = working_mode
        self.ip = ip
        # TODO check port if multiple drones are connected
        self.port = port
        self.command_socket = SocketCallback(self.ip, self.port)
        self.command_socket.add_callback(self.handle_command)
        self.command_socket.start()
        self.cam = Camera(working_mode)

    def close(self):
        print("closing python listener")
        self.command_socket.close()
        self.cam.close()
        global main_running
        main_running = False
        exit(0)

    def handle_command(self, sock, data):
        # decode the data and remove \r\n
        # TODO try to use getattribute to clean the code.
        # getattr(Camera, "foo")()
        command = data.decode().rstrip()
        print("getting command: " + command)
        if command == 'start_camera':
            self.cam.start()
            sock.send("ACK\n".encode())
        elif command == 'stop_camera':
            self.cam.stop_camera()
            sock.send("ACK\n".encode())
        elif command == 'find_target':
            sock.send("ACK\n".encode())
            found = self.cam.find_target()
            # TODO fix when camera does not find target immediatly, program does not receive any messages anymore
            if found:
                print("send found")
                sock.send("FOUND\n".encode())
            else:
                sock.send("NOT_FOUND\n".encode())
        elif command == 'exit':
            sock.send("ACK\n".encode())
            self.close()
        else:
            sock.send("NACK\n".encode())


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
    print("ip address: " + ip)
    print("port: " + str(port))
    listener = listener(working_mode, ip, port)

    while main_running:
        time.sleep(1)
