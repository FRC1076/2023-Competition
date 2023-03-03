import socket
#
#  Create the infra for two-way communication channel using UDP
#  Set receive from timeout to .001 seconds to avoid blocking for
#  long.
#
class UDPChannel:
    """
    Create a communication channel to send and receive messages
    between two addresses and ports.
    Defaults are loopback address with specific port address.
    timeout_in_seconds is the receive_from time out value.
    There is a generic exception handled for any failure related to creating
    the sending and or receiving sockets.
    """
    default_local_port = 5888
    default_remote_port = 5880
    default_local_address = '127.0.0.1'
    default_remote_address =  '127.0.0.1'

    # Useful defaults permit minimal arguments for simple test.
    # On one end:
    #      from lib1076.udp_channel import UDPChannel as UDPChannel
    #      sender = UDPChannel()
    # On the other end (in another window/program)
    #      from lib1076.udp_channel import UDPChannel as UDPChannel
    #      receiver = UDPChannel(local_port=UDPChannel.default_remote_port, remote_port=UDPChannel.default_local_port)
    def __init__(self,
                 local_ip=default_local_address,
                 local_port=default_local_port,
                 remote_ip=default_remote_address,
                 remote_port=default_remote_port,
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

