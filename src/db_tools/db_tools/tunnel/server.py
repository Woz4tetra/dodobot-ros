import time
import rospy
import queue
import socket
import select
import logging
import threading

from .protocol import TunnelProtocol


class TunnelServer:
    def __init__(self, address, port, categories):
        self.address = address
        self.port = port
        self.device = None

        self.categories = categories

        self.protocol = TunnelProtocol()

        self.inputs = []
        self.outputs = []
        self.message_queues = {}

        self.poll_timeout = 1.0
        self.read_block_size = 4096

        self.buffer = b''

    def start(self):
        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.setblocking(0)
        self.device.bind((self.address, self.port))
        self.device.listen(5)
        self.inputs.append(self.device)
    
    def poll_socket(self):
        if len(self.inputs) == 0:
            return False

        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                connection, client_address = stream.accept()
                connection.setblocking(0)
                self.inputs.append(connection)
                self.message_queues[connection] = queue.Queue()
                rospy.loginfo("Registering client %s:%s" % client_address)
                if connection not in self.outputs:
                    self.outputs.append(connection)
            else:
                recv_msg = stream.recv(self.read_block_size)
                self.buffer += recv_msg
                if recv_msg:
                    if len(self.buffer) > 0:
                        rospy.logdebug("Buffer: %s" % repr(self.buffer))
                        self.buffer, results = self.protocol.parse_buffer(self.buffer, self.categories)
                        for result in results:
                            error_code, recv_time, data = result
                            category = data[0]
                            data = data[1:]
                            if category == "__msg__":
                                rospy.loginfo("Tunnel message: %s" % data[0])
                            else:
                                self.packet_callback(error_code, recv_time, category, data)
                    # self.message_queues[stream].put(recv_msg)  # echo back to client
                    
                else:
                    self.close_stream(stream)

        for stream in writable:
            try:
                next_msg = self.message_queues[stream].get_nowait()
            except KeyError:
                continue
            except queue.Empty:
                continue
            try:
                rospy.logdebug("Writing: %s" % next_msg)
                stream.send(next_msg)
            except ConnectionResetError:
                self.close_stream(stream)

        for stream in exceptional:
            self.close_stream(stream)
    
    def write(self, category, *args):
        serialized_args = ", ".join(map(str, args))
        
        for message_queue in self.message_queues.values():
            if message_queue.full():
                rospy.logdebug("Discarding write (%s, %s). Queue is full." % (category, serialized_args))
                return
            rospy.logdebug("Creating packet from args: (%s, %s)" % (category, serialized_args))
            packet = self.protocol.make_packet(category, *args)

            rospy.logdebug("Queueing packet: %s" % repr(packet))
            message_queue.put(packet)

    def close_stream(self, stream):
        self.inputs.remove(stream)
        if stream in self.outputs:
            self.outputs.remove(stream)
        stream.close()
        del self.message_queues[stream]

    def update(self):
        try:
            self.poll_socket()
        except BaseException as e:
            rospy.logerr(str(e))
    
    def packet_callback(self, error_code, recv_time, category, data):
        pass

    def stop(self):
        self.device.close()
