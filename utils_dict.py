import numpy as np
from multiprocessing import shared_memory
import json

class SharedMemoryUtils:
    @staticmethod
    def create_shared_memory(name, size = 1024):
        try:
            # cleanup
            try:
                existing_shm = shared_memory.SharedMemory(name= name)
                existing_shm.close()
                existing_shm.unlink()
            except (FileNotFoundError, Exception):
                pass
            shm = shared_memory.SharedMemory(create=True, size = size, name = name)
            return shm
        except Exception as e:
            print(f"Exception create: {e}")
            return None

    @staticmethod
    def write(shm_name, data):
        shm = None
        try:
            json_str = json.dumps(data)
            shm = shared_memory.SharedMemory(name = shm_name)
            json_bytes = json_str.encode('utf-8')


            # buffer = shm.buf[:len(json_bytes)]
            # buffer[:] = json_bytes

            buf = shm.buf
            print(len(buf))
            if len(json_bytes) > len(buf):
                print("Data exceed")
                shm.close()
                return False
            buf[:len(json_bytes)] = json_bytes
            del buf

            shm.close()
            return True
        except Exception as e:
            print(f"Exception Writting: {e}")
            return False

    @staticmethod
    def read(shm_name):
        shm = None
        try:
            shm = shared_memory.SharedMemory(name = shm_name)
            json_bytes = bytes(shm.buf).rstrip(b'\x00')
            json_str = json_bytes.decode('utf-8')
            data = json.loads(json_str)

            # buf = shm.buf
            # json_bytes = bytes(buf).rstrip(b'\x00')
            # del buf
            # json_str = json_bytes.decode('utf-8')
            # data = json.loads(json_str)

            shm.close()
            return data
        except Exception as e:
            print(f"Exception Reading: {e}")
            return None

    @staticmethod
    def cleanup(shm_name):
        try:
            shm = shared_memory.SharedMemory(name = shm_name)
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            print(f"Memory '{shm_name}' not found")
