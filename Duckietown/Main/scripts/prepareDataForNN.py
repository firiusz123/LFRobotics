#!/usr/bin/env python3
'''
Script export from rosbag file data as car_cmd.v, car_cmd.omega and image from camera
Usage: prepareDataForNN <rosbag_file 1> <rosbag_file 2> <imag save path>
'''
import rosbag
import sys
from dataclasses import dataclass
from cv_bridge import CvBridge
import cv2
import os
import csv

# Check input parameters
try:
    rosbag_path_left    = sys.argv[1]
    rosbag_path_right   = sys.argv[2]
    data_save_path      = sys.argv[3]
except Exception:
    print("prepareDataForNN <rosbag_file-left> <rosbag_file-right>")
    exit(0)

@dataclass
class CarCmdImage:
    v: float
    omega: float
    image_name : str = None

# Read cmd_car messages
car_cmd_messages = {}
# Image counter
img_count = 0

# Set topic to prepare
car_cmd_topic =  '/' + os.environ['VEHICLE_NAME'] + '/data_sync_node/out/car_cmd'
image_topic = '/' + os.environ['VEHICLE_NAME'] + '/data_sync_node/out/image/compressed'

for rosbag_path in [rosbag_path_left, rosbag_path_right]:
    with rosbag.Bag(rosbag_path, 'r') as bag:
        
        # Iterate by car_cmd messages and creat dist of data
        for topic, msg, t in bag.read_messages(topics=[car_cmd_topic]) :
            car_cmd_messages[msg.header.stamp] = CarCmdImage( v=msg.v, omega=msg.omega)

        # Image format name
        img_file_number_name_format = "%0{}d.png".format(len(str(len(car_cmd_messages))))
        bridge = CvBridge()

        # Create data folder
        os.makedirs(data_save_path + "/img", exist_ok=True)

        # Iterate by image messages and save in file images which has equivalent car_cmd messages
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            
            if car_cmd_messages.get(msg.header.stamp)!=None:
                try:
                    # Set image name
                    image_file_name = img_file_number_name_format % img_count
                    # Save image in folder
                    cv2.imwrite(data_save_path + "/img/" + image_file_name, bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8'))
                    img_count+=1
                    # Set image name for car_cmd message
                    car_cmd_messages[msg.header.stamp].image_name = "img/" + image_file_name
                except Exception as e:
                    print("Error: {0}".format(e))
    
# Iterate by car_cmd dictionary and crete CSV file 
# filename, v, omega
with open(data_save_path + "/data.csv", 'w', encoding='UTF8') as f:
    
    # Create CSV writer handler
    csv_writer = csv.writer(f)

    # Save header
    csv_writer.writerow(['filename', 'v', 'omega'])

    for car_cmd_image in car_cmd_messages.values():
        if car_cmd_image.image_name!=None:
            csv_writer.writerow([ car_cmd_image.image_name, car_cmd_image.v, car_cmd_image.omega ])

# Summary
print("Exported: {} messages to directory: {}".format( img_count,  data_save_path))

# Compress files
os.system('tar -cf ' + data_save_path + '/data_nn.tar -C ' + data_save_path + ' img/ data.csv')

# Update permission to file and directory
os.system('chown -R ' + os.environ['USER_ID'] + ':' + os.environ['GROUP_ID'] + ' ' + data_save_path)
