import threading
import subprocess
import os
import signal
import sys

ROOT_DIR = "/home/matsya/ros2_ws/src/Robosub_ROS2_2024/auv_drivers/auv_drivers/"

def run_router():
    router_path = os.path.join(ROOT_DIR, "router.py")
    process = subprocess.Popen(["python3", router_path])
    process.wait()

def run_usb():
    usb_path = os.path.join(ROOT_DIR, "usb.py")
    process = subprocess.Popen(["python3", usb_path])
    process.wait()

def drivers():
    process = subprocess.Popen(["ros2", "launch", "auv_drivers", "drivers.launch.py"])
    process.wait()

# Create threads
os.system('sudo chmod a+rwx /dev/tty*')
run_router_thread = threading.Thread(target=run_router, daemon=True)
run_usb_thread = threading.Thread(target=run_usb, daemon=True)
drivers_thread = threading.Thread(target=drivers, daemon=True)

# Start threads
run_router_thread.start()
run_usb_thread.start()
drivers_thread.start()

# Handle Ctrl + C to terminate all threads
# def signal_handler(sig, frame):
#     print("\nStopping all processes...")
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)

# Keep main thread alive
run_router_thread.join()
run_usb_thread.join()
drivers_thread.join()
