# reader.py
from multiprocessing import shared_memory
import numpy as np
import time

def read_data(shm_name):
    try:
        existing_shm = shared_memory.SharedMemory(name=shm_name)
        shared_array = np.ndarray((5,), dtype=np.int64, buffer=existing_shm.buf)
        print("Read:", shared_array[:])
        existing_shm.close()
    except FileNotFoundError:
        print("Cannot find shared memory")

if __name__ == "__main__":
    try:
        while True:
            read_data('my_shared_memory')
            time.sleep(1) 
    except KeyboardInterrupt:
        print("\nStop reading")