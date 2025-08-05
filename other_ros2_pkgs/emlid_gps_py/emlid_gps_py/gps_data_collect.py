from collections import deque
import threading
import serial
import io
import pynmea2


######## Constants ############

address = "/dev/ttyACM0"
run_flag = True

####### Create connection and queues and start thread functions ##########
class GPSData:
    def __init__(self, addr=address):
        self.addr = addr
        self.run_flag = True
        self.thread = None
        self.channel = self.create_queues()
        self.con = None
        self.sio = None

    def create_connection(self):
        con = serial.Serial(self.addr, baudrate=57600, timeout=0.010)
        sio = io.TextIOWrapper(io.BufferedRWPair(con, con))
        if con.is_open:
            con.reset_input_buffer()
            print("Connection established with:", self.addr)
            return con, sio
        print("Connection not established with:", self.addr)
        print(
            "Are you a part of the dialout group? or have permission to access this file?")
        return None

    def create_queues(self):
        return deque(maxlen=1)

    def get_data(self):
        self.con, self.sio = self.create_connection()
        print("got connection")
        if self.con is None:
            print("broken")
            return 1

        # Create loop dependant on run flag
        while self.run_flag:
            try:
                data = self.sio.readline()
                # print("data:",data)
                msg = pynmea2.parse(data)
                # print("msg:", repr(msg))
                self.channel.append(msg)
            except Exception:
                continue
        self.con.close()

    def run(self):
        self.thread = threading.Thread(target=self.get_data, args=())
        self.thread.start()

    def close_connection(self):
        print("Closing connection")
        self.run_flag = False
        self.thread.join()
        print("Closed up")

    def get_NMEA_string(self):
        try:
            value = self.channel.pop()
        except IndexError:
            value = None
        return value
