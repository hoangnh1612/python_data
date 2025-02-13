from utils_dict import SharedMemoryUtils
import time
name = "test1"
freq = 10
def main():
    while(1):
        try:
            data = {"Name": " A",
                    "Age" : 22,
                    "Gender" : "Male", "Nationality" : "Vietnamese"}
            shm = SharedMemoryUtils.create_shared_memory(name)
            if(SharedMemoryUtils.write(name, data)):
                print(f"Sending: {data}")
                time.sleep(1/freq)
        except KeyboardInterrupt:
            print("Stop\n")
            SharedMemoryUtils.cleanup(name)
            return None


if __name__ == "__main__":
    main()
