#!/usr/bin/env python3
import sys
import subprocess

def main():
    # Filter out ROS-specific arguments
    print(sys.argv[1:])
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
    
    # Append the filtered arguments to the command
    command = ['/mnt/nvme/MWDig/workspace/adnav_ws/install/ros2-driver/lib/ros2-driver/adnav_driver'] + filtered_args
    
    # Execute the command
    subprocess.run(command)

if __name__ == '__main__':
    main()