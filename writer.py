# writer.py
from multiprocessing import shared_memory
import numpy as np
import time

def write_data():
    try:
        shm = shared_memory.SharedMemory(name="my_shared_memory")
    except FileNotFoundError:
        shm = shared_memory.SharedMemory(create=True, size=5*8, name="my_shared_memory")
    
    data = np.array([1, 2, 3, 4, 5])
    shared_array = np.ndarray((5,), dtype=np.int64, buffer=shm.buf)
    shared_array[:] = data[:]
    print("Wrote data:", shared_array[:])
    return shm.name

if __name__ == "__main__":
    try:
        while True:
            write_data()
            time.sleep(1) 
    except KeyboardInterrupt:
        shm = shared_memory.SharedMemory(name="my_shared_memory")
        shm.close()
        shm.unlink()