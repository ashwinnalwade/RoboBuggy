import serial;
import sys
import struct
import time

class RBSerialMessage:

    def __init__(self, device_path):
        self.RBSM_FOOTER = "\n" # 0x0A
        self.RBSM_BAUD_RATE = 57600
        self.RBSM_PACKET_LENGTH = 6
        # packet format: unsigned byte, big-endian signed int, char
        self.RBSM_PACKET_FORMAT = ">Bic"

        self.device_path = device_path
        self.port = serial.Serial(device_path, self.RBSM_BAUD_RATE)

        self.stream_lock = False

    def read(self):
        # if we don't have a stream lock read until we find a footer
        if(self.stream_lock == False):
            possible_footer = self.port.read(1)
            while(possible_footer != self.RBSM_FOOTER):
                possible_footer = self.port.read(1)
            self.stream_lock = True

        # capture a packet length
        message_buffer = ""
        message_unpacked = None
        message_id = None
        message_data = None

        # read what we think is a full packet
        for i in range(self.RBSM_PACKET_LENGTH):
            message_buffer += self.port.read(1)

        # unpack the packet
        message_unpacked = struct.unpack_from(self.RBSM_PACKET_FORMAT, message_buffer)


        # confirm properly formed packet
        if(message_unpacked[2] == self.RBSM_FOOTER):
            return {"id": message_unpacked[0],
                            "data": message_unpacked[1],
                            "status": "locked"}

        # but unlock the stream if we can't find a footer
        else:
            self.stream_lock = False
            return {"id": 0, "data": 0, "status": "unlocked"}

    def send(self, message_id, message_data):
        message_buffer = struct.pack(self.RBSM_PACKET_FORMAT,
                                                                 message_id,
                                                                 message_data,
                                                                 self.RBSM_FOOTER)
        self.port.write(message_buffer)
        self.port.flush()


if __name__ == "__main__":
    import time
    # create a test rbsm channel
    rbsm_endpoint = RBSerialMessage(sys.argv[1])
    print("waiting for reset...")
    time.sleep(2)
    # arduino resets on connection. be careful to not miss messages

    increment = 10
    i = -1000
    while(True):
        i += increment
        if ((i >= 1000) or (i <= -1000)):
            increment = -increment

        rbsm_endpoint.send(20, i)
        time.sleep(0.01)
        print(i)    



    print("send messages successfully.")
    
    # then listen for messages forever
    # print("here are some messages I'm seeing:")
    # while(1):
    #   message = rbsm_endpoint.read()
    #   print (message)

