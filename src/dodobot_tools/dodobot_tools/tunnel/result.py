from .util import *

class PacketResult:
    def __init__(self, error_code, recv_time):
        self.error_code = error_code
        self.recv_time = recv_time

        self.category = ""
        self.buffer = b''
        self.start_index = 0
        self.stop_index = 0
        self.current_index = 0
    
    def set_start_index(self, index):
        self.start_index = index
        self.current_index = self.start_index
    
    def set_stop_index(self, index):
        self.stop_index = index

    def set_buffer(self, buffer):
        self.buffer = buffer
    
    def set_error_code(self, error_code):
        if error_code == NULL_ERROR:
            return
        self.error_code = error_code
    
    def set_category(self, category):
        self.category = category

    def check_index(self):
        if self.current_index >= self.stop_index:
            raise RuntimeError("Index exceeds buffer limits. %d >= %d" % (self.current_index, self.stop_index))
    
    def get_int(self):
        next_index = self.current_index + 4
        result = to_int(self.buffer[self.current_index: next_index])
        self.current_index = next_index
        self.check_index()
        return result

    def get_double(self):
        next_index = self.current_index + 8
        result = to_float(self.buffer[self.current_index: next_index])
        self.current_index = next_index
        self.check_index()
        return result

    def get_string(self, length=None):
        if length is None:
            next_index = self.current_index + 2
            length = to_int(self.buffer[self.current_index: next_index])

        next_index = self.current_index + length
        result = self.buffer[self.current_index: next_index].decode()
        self.current_index = next_index
        self.check_index()
        return result
