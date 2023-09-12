#! /usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import message_filters
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

import os
import sys
import time
import datetime
import numpy as np
import termios, tty
from pynput.keyboard import Key, Listener

import simpleaudio as sa

name = "subject_024"
# stage = "baseline_familiarization"
# stage = "baseline"
# stage = "controlled_familiarization"
stage = "controlled"

# changes not 

class myDataLogger():
	def __init__(self):
		rospy.init_node('data_collection_node', anonymous=True, disable_signals=False)
		self.wave_obj = sa.WaveObject.from_wave_file('/home/kia/catkin_ws/src/data_collection_human_test/assets/beep-07a.wav')
		self.settings = termios.tcgetattr(sys.stdin)

		while input("press enter to start saving data, or type ctrl c then n to not: ") != "n":
			self.stop         = False
			self.subscriber_flag = False
			self.xelaSensor1  = []
			self.hand_imu	  = []
			self.objet_imu	  = []
			self.object_marker = []
			self.fixed_marker  = []
			self.t0 = 0
			self.t1 = 0
			self.t2 = 0
			self.rate = 0

			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()
			
			self.hand_imu_sub      = message_filters.Subscriber('/hand_imu', Float64MultiArray)
			self.object_imu_sub    = message_filters.Subscriber('/object_imu', Float64MultiArray)
			self.xela_sensor_sub   = message_filters.Subscriber('/xela1_data', Float64MultiArray)
			self.marker_object_sub = message_filters.Subscriber('/aruco_simple/poseStamped', PoseStamped)
			self.marker_fixed_sub  = message_filters.Subscriber('/aruco_simple/poseStamped2', PoseStamped)
			subscribers = [self.hand_imu_sub, self.object_imu_sub, self.xela_sensor_sub, self.marker_object_sub, self.marker_fixed_sub]
			self.start_time = datetime.datetime.now()
			
			self.counter = 0
			self.prev_i = 0
			self.task_start_time_step = 0
			self.task_middle_time_step = 0
			self.task_end_time_step = 0
			self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=1, slop=0.1, allow_headerless=True)
			self.ts.registerCallback(self.read_data)
			rate = rospy.Rate(1000)
			
			self.data_collection_start_time = time.time()
			while (not rospy.is_shutdown()) and (self.stop is False):
				loop_time = time.time()

				if self.prev_i!=0 and self.object_marker and 9 > self.object_marker[-1][0] > -0.49 and self.t0 == 0:
					# save the motion start time and index 
					self.t0 = loop_time
					self.task_start_time_step = self.counter
				elif self.prev_i!=0 and self.object_marker and 9 > self.object_marker[-1][0] > self.fixed_marker[-1][0] and self.t1 == 0:
					# save the forward motion end time and index 
					self.t1 = loop_time
					self.task_middle_time_step = self.counter
				elif self.prev_i!=0 and self.object_marker and self.object_marker[-1][0] < -0.49 and self.t0!=0 and self.t2==0:
					# save the motion end time and index
					self.t2 = loop_time
					self.task_end_time_step = self.counter	

				
				if 1.99 < (loop_time - self.data_collection_start_time) < 2.01:
					play_obj = self.wave_obj.play()
					play_obj.wait_done()
				elif 5.99 < (loop_time - self.data_collection_start_time) < 6.01:
					play_obj = self.wave_obj.play()
					play_obj.wait_done()
				elif 7.99 < (loop_time - self.data_collection_start_time) < 8.01:
					play_obj = self.wave_obj.play()
					play_obj.wait_done()
				elif self.t2!=0 and 0.99 < (loop_time - self.t2) < 1.01:
					play_obj = self.wave_obj.play()
					play_obj.wait_done()
				
				# if self.t0 != 0 and 0.39999555 < (loop_time - self.t0) < 0.4020000:
				# 	play_obj = self.wave_obj.play()
				# 	play_obj.wait_done()
				# elif self.t0 != 0 and 0.799955555 < (loop_time - self.t0) < 0.80200000:
				# 	play_obj = self.wave_obj.play()
				# 	play_obj.wait_done()

				self.prev_i += 1
				rate.sleep()
			
			self.task_completion_time = self.t2 - self.t0
			# self.end_subscription()
			self.stop = False
			self.stop_time = datetime.datetime.now()
			
			if self.t0!=0 and self.t2!=0:
				self.rate = (len(self.xelaSensor1[self.task_start_time_step:self.task_end_time_step])) / (self.t2 - self.t0)
			print("\n Stopped the data collection \n now saving the stored data")
			self.listener.stop()
			self.save_data()

	def read_data(self, hand_imu_data, object_imu_data, xela_data, object_marker_data, fixed_marker_data):
		self.counter += 1
		self.subscriber_flag = True
		self.hand_imu.append(hand_imu_data.data)
		self.objet_imu.append(object_imu_data.data)
		self.xelaSensor1.append(xela_data.data)
		self.object_marker.append([object_marker_data.pose.position.x, object_marker_data.pose.position.y, object_marker_data.pose.position.z,
								object_marker_data.pose.orientation.x, object_marker_data.pose.orientation.y, object_marker_data.pose.orientation.z,
								object_marker_data.pose.orientation.w])
		self.fixed_marker.append([fixed_marker_data.pose.position.x, fixed_marker_data.pose.position.y, fixed_marker_data.pose.position.z,
								fixed_marker_data.pose.orientation.x, fixed_marker_data.pose.orientation.y, fixed_marker_data.pose.orientation.z,
								fixed_marker_data.pose.orientation.w])
		
	def end_subscription(self):
		self.listener.stop()
		self.hand_imu_sub.unregister()
		self.object_imu_sub.unregister()
		self.xela_sensor_sub.unregister()
		self.marker_object_sub.unregister()
		self.marker_fixed_sub.unregister()

	def start_collection(self, key):
		print("herer")
		if key == Key.esc:
			self.stop = True
			self.listener.stop()
			self.hand_imu_sub.unregister()
			self.object_imu_sub.unregister()
			self.xela_sensor_sub.unregister()
			self.marker_object_sub.unregister()
			self.marker_fixed_sub.unregister()

	def save_data(self):
		self.hand_imu      = np.array(self.hand_imu)
		self.objet_imu     = np.array(self.objet_imu )
		self.xelaSensor1   = np.array(self.xelaSensor1)
		self.object_marker = np.array(self.object_marker)
		self.fixed_marker  = np.array(self.fixed_marker)
		T1 = pd.DataFrame(self.hand_imu)
		T2 = pd.DataFrame(self.objet_imu)
		T3 = pd.DataFrame(self.xelaSensor1)
		T4 = pd.DataFrame(self.object_marker)
		T5 = pd.DataFrame(self.fixed_marker)

		self.folder = str('/home/kia/catkin_ws/src/data_collection_human_test/data/real_test2/' + name + '/' + stage + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		mydir = os.mkdir(self.folder)

		imu_save_col = ["q0", "q1", "q2", "q3", "acc_x", "acc_y", "acc_z"]
		xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
				'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
				'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

		marker_pose_col = ["marker_position_x", "marker_position_y", "marker_position_z", 
							"marker_quaternion_x", "marker_quaternion_y", "marker_quaternion_z", "marker_quaternion_w"]

		T1.to_csv(self.folder + '/hand_imu.csv', header=imu_save_col, index=False)
		T2.to_csv(self.folder + '/object_imu.csv', header=imu_save_col, index=False)
		T3.to_csv(self.folder + '/xela.csv', header=xela_Sensor_col, index=False)
		T4.to_csv(self.folder + '/object_marker.csv', header=marker_pose_col, index=False)
		T5.to_csv(self.folder + '/fixed_marker.csv', header=marker_pose_col, index=False)

		print("Frequency: ", self.rate)

		# Create meta data
		meta_data = ['user', 'gender']
		meta_data_ans = [] # ["1", "0", "1", "1", "NOT KINESTHETIC"]
		for info in meta_data:
			value = input(str("please enter the " + info))
			meta_data_ans.append(value)
		meta_data.extend(('frequency_hz', 'start_time', 'stop_time', 'task_completion_time', 'start_index', 'middle_index', 'end_index'))
		meta_data_ans.extend((str(self.rate), str(self.start_time), str(self.stop_time), str(self.task_completion_time), str(self.task_start_time_step), str(self.task_middle_time_step), str(self.task_end_time_step)))
		meta_data_ans = np.array([meta_data_ans])
		T5 = pd.DataFrame(meta_data_ans)
		T5.to_csv(self.folder + '/meta_data.csv', header=meta_data, index=False)

if __name__ == "__main__":
	data_reader = myDataLogger()
