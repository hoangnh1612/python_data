# writer.py
from multiprocessing import shared_memory
import numpy as np
import time

def write_data():
    i = 1
    try:
        shm = shared_memory.SharedMemory(name="my_shared_memory")
    except FileNotFoundError:
        shm = shared_memory.SharedMemory(create=True, size=4*8, name="my_shared_memory")
    
    timestamp = time.time()
    data = np.array([timestamp, i, i+1, i+2])
    print(data.nbytes)
    shared_array = np.ndarray((4,), dtype=np.float64, buffer=shm.buf)
    shared_array[:] = data[:]
    print(f"Wrote at {timestamp:.6f}: {shared_array[1:]}")
    i += 1
    return shm.name

if __name__ == "__main__":
    try:
        while True:
            write_data()
            # time.sleep(1)
    except KeyboardInterrupt:
        shm = shared_memory.SharedMemory(name="my_shared_memory")
        shm.close()
        shm.unlink()