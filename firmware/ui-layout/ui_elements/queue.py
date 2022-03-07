class Queue:
    def __init__(self, max_len):
        self._max_len = max_len
        self._buffer = [None for _ in range(self._max_len)]

        self._len = 0
        self._back = -1
        self._front = 0

    def empty(self):
        return self._len == 0

    def full(self):
        return self._len == self._max_len

    def size(self):
        return self._len

    def queue(self, item):
        if self.full():
            return False

        if self._back == self._max_len - 1:
            self._back = -1

        self._back += 1
        self._buffer[self._back] = item

        self._len += 1
        return True

    def deque(self):
        if self.empty():
            return -1

        index = self._front
        self._front += 1

        if self._front == self._max_len:
            self._front = 0
        self._len -= 1
        return index

    def peek(self):
        if self.empty():
            return -1
        return self._front

    def get(self, index):
        if index < 0:
            index = 0
        if index >= self._max_len:
            index = self._max_len - 1
        return self._buffer[index]


class NumberSequence(Queue):
    def __init__(self, max_len):
        super(NumberSequence, self).__init__(max_len)

    def check_sequence(self, sequence):
        if self.empty():
            return False
        if len(sequence) > self._len:
            return False

        index = self._front
        seq_index = 0
        while index != self._back:
            if self._buffer[index] == sequence[seq_index]:
                print("sequence match at", index)
                seq_index += 1
            else:
                seq_index = 0
                print("Resetting. sequence mismatch at [%s]=%s, [%s]=%s" % (index, self._buffer[index], seq_index, sequence[seq_index]))
                if self._buffer[index] == sequence[seq_index]:
                    print("sequence match at", index)
                    seq_index += 1

            index += 1
            if index >= self._max_len:
                print("looping")
                index = 0
            if seq_index >= len(sequence):
                break

        if seq_index == len(sequence) - 1:
            return self._buffer[index] == sequence[seq_index]
        return False

    def print_front_to_back(self):
        index = self._front
        while index != self._back:
            print(self._buffer[index], end=", ")
            index += 1
            if index >= self._max_len:
                index = 0
        print(self._buffer[index])
        print(self._buffer)

if __name__ == '__main__':
    def on_numpad(number, queue, sequence):
        if queue.full():
            queue.deque()
        queue.queue(number)

        print()
        queue.print_front_to_back()
        print(queue._front, queue._back)

        if queue.check_sequence(sequence):
            while not queue.empty():
                queue.deque()
            print("sequence found!")


    def test_seq():
        num_queue = NumberSequence(10)
        sequence = [8, 1, 5, 3, 8]
        on_numpad(1, num_queue, sequence)
        on_numpad(1, num_queue, sequence)
        on_numpad(1, num_queue, sequence)
        on_numpad(1, num_queue, sequence)
        on_numpad(1, num_queue, sequence)
        on_numpad(8, num_queue, sequence)
        on_numpad(1, num_queue, sequence)

        for _ in range(3):
            on_numpad(8, num_queue, sequence)
            on_numpad(1, num_queue, sequence)
            on_numpad(5, num_queue, sequence)
            on_numpad(3, num_queue, sequence)
            on_numpad(8, num_queue, sequence)
            # on_numpad(8, num_queue, sequence)
    test_seq()

