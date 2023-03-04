import socket
import json

class LEDSender: 
    def __init__(self, _local_ip, _local_port, _remote_port, _remote_ip):
        self.sender = UDPChannel(
            local_ip = _local_ip,
            local_port = _local_port,
            remote_port = _remote_port,
            remote_ip = _remote_ip)

    def send_logo(self, num):
        data = {
            'sender' : 'driver',
            'message' : 'logo',
            'logo' : num
        }

        message = json.dumps(data)
        self.sender.send_to(message)


class UDPChannel:
    """
    Create a communication channel to send and receive messages
    between two addresses and ports.
    Defaults are loopback address with specific port address.
    timeout_in_seconds is the receive_from time out value.
    There is a generic exception handled for any failure related to creating
    the sending and or receiving sockets.
    """

    def __init__(self,
                 local_ip,
                 local_port,
                 remote_ip,
                 remote_port,
                 timeout_in_seconds=.0001,
                 receive_buffer_size=8192):
        """
        Create the sending and receiving sockets for a communcation channel
        If the address for the local end of the channel is not valid
        this will throw an exception.    The user should retry creating
        the channel later.
        """
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port

        # cache other configurable parameters
        self.timeout_in_seconds = timeout_in_seconds
        self.receive_buffer_size = receive_buffer_size

        # create the receive socket
        self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receive_socket.bind((local_ip, local_port))

        # and the sending socket
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_to(self, message):
            """
            Send message to the other end of the channel.
            Send to the configured address and port
            """
            self.send_socket.sendto(message.encode(), (self.remote_ip, self.remote_port))
    
    def reply_to(self, message, ip, port):
            """
            Reply to a message received from the specified (ip, port)
            Encode the string for sending.
            """
            self.send_socket.sendto(message.encode(), (ip, port))

    def receive_reply(self):
            """receive a reply"""
            self.send_socket.settimeout(self.timeout_in_seconds)
            return self.send_socket.recvfrom(self.receive_buffer_size)

    def receive_from(self):
            """
            wait for timeout to receive a message from channel
            If there is a timeout, return None,None
            Otherwise return (decoded_message, addr_n_port)
            """
            self.receive_socket.settimeout(self.timeout_in_seconds)
            try:
                (message, portaddr) = self.receive_socket.recvfrom(self.receive_buffer_size)
            except socket.timeout:
                # if we timed out, we got nothing
                (message, portaddr) = (None, None)
            except Exception as unknown:
                print('Problem receiving UDP packet"',unknown)
                (message, portaddr) = (None, None)
            else:
                # if it worked, we must decode
                message = message.decode()
            return (message, portaddr)

    def close(self):
        self.receive_socket.close()
        self.send_socket.close()

