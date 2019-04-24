from Camera import Camera
import argparse
import time

from SocketCallback import SocketCallback
from Logging import Logger as log

main_running = True


class listener:

    logger = log.logger

    def __init__(self, working_mode, ip, port):
        self.logger.info("\n\nNEW START")
        self.logger.info("init listener")
        self.working_mode = working_mode

        self.ip = ip
        # TODO check port if multiple drones are connected
        self.port = port
        self.logger.info("working mode: " + working_mode + " ip addres: " + ip + " port: " + str(port))
        self.command_socket = SocketCallback(self.ip, self.port)
        self.command_socket.add_callback(self.handle_command)
        self.command_socket.start()

        self.cam = Camera(working_mode)

    def close(self):
        self.logger.info("closing listener")
        self.command_socket.close()
        self.cam.close()
        global main_running
        main_running = False
        exit(0)

    def handle_command(self, sock, data):
        # decode the data and remove \r\n
        command = data.decode().rstrip()
        self.logger.info("getting command: " + command)
        success = self.cam.process_command(command)

        if success == 1:
            sock.send("ACK\n".encode())
            if command == 'exit':
                self.close()
        elif success == 0:
            sock.send("NACK\n".encode())
        elif success == -1:
            sock.send("NACK\n".encode())
            # TODO do something when exception accours


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-w", "--working_mode", help="choice working mode <pc or pi>")
    parser.add_argument("-i", "--ip", help="ip address of host for socket with java")
    parser.add_argument("-p", "--port", help="port for socket with java")
    args = parser.parse_args()

    working_mode = "pi" if not args.working_mode else args.working_mode
    port = 5761 if not args.port else int(args.port)
    ip = "localhost" if not args.ip else args.ip

    listener = listener(working_mode, ip, port)

    while main_running:
        time.sleep(1)
