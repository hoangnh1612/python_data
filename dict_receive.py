from utils_dict import SharedMemoryUtils
import time
name = "test3"
freq = 10
def main():
    while(1):
        try:
            data = SharedMemoryUtils.read(name)
            if data:
                print(f"Received: {data}")
            else:
                print("No data received")
            time.sleep(1/freq)
            
        except KeyboardInterrupt:
            print("Stop Receiving")
        # finally:
            SharedMemoryUtils.cleanup(name)
            return None
if __name__ == "__main__":
    main()