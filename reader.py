# reader.py
from multiprocessing import shared_memory
import numpy as np
import time

def read_data(shm_name):
    try:
        existing_shm = shared_memory.SharedMemory(name=shm_name)
        shared_array = np.ndarray((4,), dtype=np.float64, buffer=existing_shm.buf)
        current_time = time.time()
        write_time = shared_array[0]
        latency = (current_time - write_time) * 1000  
        print(f"Read at {current_time:.6f}, Data: {shared_array[1:]}, Latency: {latency:.3f}ms")
        existing_shm.close()
    except FileNotFoundError:
        print("Cannot find shared memory")

if __name__ == "__main__":
    try:
        while True:
            read_data('my_shared_memory')
            #time.sleep(1)
    except KeyboardInterrupt:
        print("\nStop reading")