#! /usr/bin/env python
import rospy
import json
import websocket
import time
from std_msgs.msg import Float64MultiArray


ip = "10.5.32.9"
port = 5000

save_list = []

rospy.init_node('xela_sensor')
xela1_pub = rospy.Publisher('/xela1_data', Float64MultiArray, queue_size = 1)
rate = rospy.Rate(100) # 100hz
time.sleep(5)

def publisher(xela_pub, data):
    xela_msg = Float64MultiArray()
    xela_msg.data = data
    xela_pub.publish(xela_msg)


def on_message(wsapp, message):
    data = json.loads(message)
    sensor1 = data['1']['calibrated']
    txls = int(len(sensor1)/3)
    data1_row = []
    
    for i in range(txls):
        
        x = sensor1[i*3]
        y = sensor1[i*3+1]
        z = sensor1[i*3+2]
        data1_row.append(x)
        data1_row.append(y)
        data1_row.append(z)


    publisher(xela1_pub, data1_row)
    rate.sleep()
       

wsapp = websocket.WebSocketApp("ws://{}:{}".format(ip,port), on_message=on_message)
wsapp.run_forever()