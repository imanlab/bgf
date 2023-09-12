#! /usr/bin/env python3
from argparse import _ActionsContainer
from os import device_encoding
import rospy
import time
import pickle
import datetime
import numpy as np
import pandas as pd
import message_filters
from pickle import load
import numpy.matlib as mat
from numpy.random import seed
from pykalman import KalmanFilter
# from xela_server.msg import XStream
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Float64
from scipy.spatial.transform import Rotation as R
from python.CppPythonSocket.server import Server

## Optimizer
from scipy.optimize import Bounds
from scipy.spatial import distance
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint

## Pytorch
import torch
import torch.onnx
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import onnx
import onnxruntime

from ACTP.ACTP_model import ACTP
# from ClassifierLSTM.classifier_model import ClassifierLSTM
from ClassifierLSTM.seq_classifier_lstm import ClassifierLSTM

from scipy.special import expit
from scipy.misc import derivative


seed = 42
context_frames = 10
sequence_length = 20
torch.manual_seed(seed)
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class RobotTest():
	def __init__(self):
		self.stop = 0.0
		self.time_step = 0
		self.prev_time_step = 0
		self.translation_x = 0.35
		self.translation_y = 0.46
		# self.previous_w = np.random.normal(0, 0.1, 5)
		self.xela_data 		      = np.zeros((1000, 48))
		self.xela_kalman_mean	  = np.zeros((1000, 48))
		self.xela_kalman_cov_x    = np.zeros((1000, 16, 16))
		self.xela_kalman_cov_y    = np.zeros((1000, 16, 16))
		self.xela_kalman_cov_z    = np.zeros((1000, 16, 16))
		self.xela_scaled_data     = np.zeros((1000, 48))
		self.xela_descaled_data   = np.zeros((1000, 48))
		self.robot_data 	      = np.zeros((1000, 6))
		self.action_data 	      = np.zeros((1000, 6))
		self.robot_data_scaled 	  = np.zeros((1000, 6))
		self.action_data_scaled   = np.zeros((1000, 6))
		self.predicted_slip_class = np.zeros((1000, 10, 1))
		self.tactile_predictions  = np.zeros((1000, 10, 48))
		self.tactile_predictions_descaled = np.zeros((1000, 10, 48))
		self.marker_data		  = np.zeros((1000, 7))

		self.optimal_weights      = np.zeros((700, 18))
		self.ref_spherical_trajectory = np.zeros((700, 10, 6))
		self.spherical_trajectory = np.zeros((700, 10, 6))
		self.optimal_trajectory   = np.zeros((700, 10, 6))
		self.opt_execution_time   = np.zeros(700)
		self.optimality           = np.zeros(700)
		self.num_itr              = np.zeros(700)
		self.constr_violation     = np.zeros(700)

		self.pred_ort_session  = onnxruntime.InferenceSession("/home/kiyanoush/Cpp_ws/src/robotTest2/ONNX_torch/6dofpred_model_onnx.onnx")
		self.class_ort_session = onnxruntime.InferenceSession("/home/kiyanoush/Cpp_ws/src/robotTest2/ONNX_torch/6dofclassification_model_onnx.onnx")
		
		self.save_results_path = "/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/031/no control"
		
		# self.robot_actions_pre_calc = np.load("/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/reference_trajectories/robot_data_2dof.npy")
		# self.robot_actions_pre_calc = np.load("/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/reference_trajectories/robot_data.npy")
		# self.robot_actions_pre_calc = np.load("/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/reference_trajectories/robot_data_newpose1.npy")
		self.robot_actions_pre_calc = np.load("/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/reference_trajectories/robot_data_cubic_1dof.npy")
		
		rospy.init_node('listener', anonymous=True, disable_signals=True)
		self.model_predict_001_init()
		self.load_scalers()
		self.init_sub()
		self.control_loop()
	
	def init_sub(self):
		sync_data_sub = message_filters.Subscriber('/sync_data', Float64MultiArray)
		self.sync_subscriber = [sync_data_sub]
		ts_sync = message_filters.ApproximateTimeSynchronizer(self.sync_subscriber, queue_size=1, slop=0.1, allow_headerless=True)
		ts_sync.registerCallback(self.sub_cb)
		self.slip_prediction_pub = rospy.Publisher('/slip_prediction', Float64, queue_size=11)
		self.optimal_traj_pub = rospy.Publisher('/optimal_traj', Float64MultiArray, queue_size=11)

	def load_scalers(self):
		self.scaler_path = '/home/kiyanoush/Cpp_ws/src/robotTest2/python scripts/6dof/scalars'
		self.robot_min_max_scalar    = [load(open(self.scaler_path + '/robot_min_max_scalar_'+feature +'.pkl', 'rb')) for feature in ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']]
		self.tactile_standard_scaler = [load(open(self.scaler_path + '/tactile_standard_scaler_'+feature +'.pkl', 'rb')) for feature in ['x', 'y', 'z']]
		self.tactile_min_max_scalar  = [load(open(self.scaler_path + '/tactile_min_max_scalar_'+feature +'.pkl', 'rb')) for feature in ['x', 'y', 'z']]
		# self.tactile_pred_standard_scaler = [load(open(self.scaler_path + '/tactile_pred_standard_scaler_'+feature+'.pkl', 'rb')) for feature in ['x', 'y', 'z']]
		self.tactile_pred_standard_scaler = load(open(self.scaler_path + '/classifier_standard_scaler.pkl', 'rb'))

	def descale_data(self, tactile):
		tp_back_scaled = []
		if tactile.shape[0] == 48:
			(tpx, tpy, tpz) = np.split(np.expand_dims(tactile, axis=0), 3, axis=1)
		else:
			(tpx, tpy, tpz) = np.split(tactile, 3, axis=1)

		xela_x_inverse_minmax = self.tactile_min_max_scalar[0].inverse_transform(tpx)
		xela_y_inverse_minmax = self.tactile_min_max_scalar[1].inverse_transform(tpy)
		xela_z_inverse_minmax = self.tactile_min_max_scalar[2].inverse_transform(tpz)
		xela_x_inverse_full = self.tactile_standard_scaler[0].inverse_transform(xela_x_inverse_minmax)
		xela_y_inverse_full = self.tactile_standard_scaler[1].inverse_transform(xela_y_inverse_minmax)
		xela_z_inverse_full = self.tactile_standard_scaler[2].inverse_transform(xela_z_inverse_minmax)
		tp_back_scaled = np.concatenate((xela_x_inverse_full, xela_y_inverse_full, xela_z_inverse_full), axis=1)

		return tp_back_scaled

	def remove_offset(self, tactile_data_init):
		tactile_data_init = tactile_data_init - self.initial_xela_row
		tactile_data_split = [np.array(tactile_data_init[:, [i for i in range(feature, 48, 3)]]).astype(float) for feature in range(3)]	
		tactile = [[tactile_data_split[feature][ts] for feature in range(3)] for ts 
																		in range(tactile_data_split[0].shape[0])] # output of shape trial_lenx3x16
		tactile = np.array(tactile)

		return tactile[0] # shape: 3x16
	
	def scale_action(self, action):
		for index, min_max_scalar in enumerate(self.robot_min_max_scalar):
			action[:, index] = np.squeeze(min_max_scalar.transform(action[:, index].reshape(-1, 1)))

		action = action[:, np.newaxis, :].astype(np.float32)

		return action

	def scale_data(self, tactile_data, action):
		new_vec = np.zeros((10, 48))
		new_vec[:, :16] = self.tactile_standard_scaler[0].transform(tactile_data[:, :16])
		new_vec[:, 16:32] = self.tactile_standard_scaler[1].transform(tactile_data[:, 16:32])
		new_vec[:, 32:48] = self.tactile_standard_scaler[2].transform(tactile_data[:, 32:48])
		
		new_vec[:, :16] = self.tactile_min_max_scalar[0].transform(new_vec[:, :16])
		new_vec[:, 16:32] = self.tactile_min_max_scalar[1].transform(new_vec[:, 16:32])
		new_vec[:, 32:48] = self.tactile_min_max_scalar[2].transform(new_vec[:, 32:48])

		for index, min_max_scalar in enumerate(self.robot_min_max_scalar):
			action[:, index] = np.squeeze(min_max_scalar.transform(action[:, index].reshape(-1, 1)))

		tactile_input = new_vec[:, np.newaxis, :].astype(np.float32)
		action = action[:, np.newaxis, :].astype(np.float32)

		return tactile_input, action
	
	def preprocess_predictions(self, tactile_prediction):
		tactile_prediction[:, :32] = self.tactile_pred_standard_scaler.transform(tactile_prediction[:, :32])
		tactile_prediction = tactile_prediction[:, np.newaxis, :32].astype(np.float32)

		return tactile_prediction

	def model_predict_001_init(self):
		# ACTP model:
		self.model = torch.load("/home/kiyanoush/Cpp_ws/src/robotTest2/python scripts/6dof/torch_models/ACTP", map_location='cpu').to(device).double()
		self.model.eval()
		# Classifier LSTM
		self.classifier = torch.load("/home/kiyanoush/Cpp_ws/src/robotTest2/python scripts/6dof/torch_models/classifier_lstm", map_location='cpu').to(device).float()
		self.classifier.eval()

	def init_kalman_filter(self, xela_vec):
		self.kf_x = KalmanFilter(initial_state_mean=xela_vec[0], n_dim_obs=16, transition_covariance=1*np.eye(16), observation_covariance=4*np.eye(16))
		self.kf_y = KalmanFilter(initial_state_mean=xela_vec[1], n_dim_obs=16, transition_covariance=1*np.eye(16), observation_covariance=4*np.eye(16))
		self.kf_z = KalmanFilter(initial_state_mean=xela_vec[2], n_dim_obs=16, transition_covariance=1*np.eye(16), observation_covariance=8*np.eye(16))

		self.xela_kalman_mean[0, 0:16], self.xela_kalman_cov_x[0] = self.kf_x.filter_update(xela_vec[0], 4*np.eye(16), xela_vec[0])
		self.xela_kalman_mean[0, 16:32], self.xela_kalman_cov_y[0] = self.kf_y.filter_update(xela_vec[1], 4*np.eye(16), xela_vec[1])
		self.xela_kalman_mean[0, 32:48], self.xela_kalman_cov_z[0] = self.kf_z.filter_update(xela_vec[2], 8*np.eye(16), xela_vec[2])
	
	def kalman_filter_func(self):
		if self.time_step == 0:
			self.init_kalman_filter(self.xela_vec)
		else:
			self.xela_kalman_mean[self.time_step, :16], self.xela_kalman_cov_x[self.time_step] = \
					self.kf_x.filter_update(self.xela_kalman_mean[self.time_step-1, :16], self.xela_kalman_cov_x[self.time_step-1], self.xela_vec[0])
			self.xela_kalman_mean[self.time_step, 16:32], self.xela_kalman_cov_y[self.time_step] = \
					self.kf_y.filter_update(self.xela_kalman_mean[self.time_step-1, 16:32], self.xela_kalman_cov_y[self.time_step-1], self.xela_vec[1])
			self.xela_kalman_mean[self.time_step, 32:48], self.xela_kalman_cov_z[self.time_step] = \
					self.kf_z.filter_update(self.xela_kalman_mean[self.time_step-1, 32:48], self.xela_kalman_cov_z[self.time_step-1], self.xela_vec[2])
			
	def add_to_save(self):
		# store the prediction data:
		self.tactile_predictions[self.time_step , :10 , : ] = self.prediction[0][:, 0, :]
		self.tactile_predictions_descaled[self.time_step , :10 , : ] = self.descale_data(self.prediction[0][:, 0, :])
		self.xela_scaled_data[self.time_step, :]   = self.scaled_tactile[-1,0]
		self.xela_descaled_data[self.time_step, :] = self.descale_data(self.scaled_tactile[-1,0])
		self.robot_data_scaled[self.time_step, :]  = self.scaled_action[ 9 , 0 , :]
		self.action_data_scaled[self.time_step, :] = self.scaled_action[ -1 , 0 , :]
		self.predicted_slip_class[self.time_step] = self.slip_pred_tag[:, :, 0]
	
	def save_results(self):
		np.save(self.save_results_path + "/xela_raw.npy", self.xela_data[:self.time_step])
		np.save(self.save_results_path + "/xela_kalman_filtered.npy", self.xela_kalman_mean[:self.time_step])
		np.save(self.save_results_path + "/xela_scaled.npy", self.xela_scaled_data[:self.time_step])
		np.save(self.save_results_path + "/xela_descaled.npy", self.xela_descaled_data[:self.time_step])
		np.save(self.save_results_path + "/tactile_prediction.npy", self.tactile_predictions[:self.time_step])
		np.save(self.save_results_path + "/tactile_prediction_descaled.npy", self.tactile_predictions_descaled[:self.time_step])
		np.save(self.save_results_path + "/slip_prediction.npy", self.predicted_slip_class[:self.time_step])
		np.save(self.save_results_path + "/robot_data.npy", self.robot_data[:self.time_step])
		np.save(self.save_results_path + "/action_data.npy", self.action_data[:self.time_step])
		np.save(self.save_results_path + "/robot_data_scaled.npy", self.robot_data_scaled[:self.time_step])
		np.save(self.save_results_path + "/action_data_scaled.npy", self.action_data_scaled[:self.time_step])
		np.save(self.save_results_path + "/marker.npy", self.marker_data[:self.time_step])

		self.save_meta_data()
	
	def save_meta_data(self):
		data_keys = ['controlled', 'trajectory', 'dof', 'object_id', 'start_point']
		meta_data_dict = dict.fromkeys(data_keys)

		meta_data_dict['controlled'] = False
		meta_data_dict['trajectory'] = 'trapezoidal'
		meta_data_dict['dof'] = 1
		meta_data_dict['object_id'] = 1
		meta_data_dict['start_point'] = 1
		
		meta_file = open(self.save_results_path + "/data.pkl", "wb")
		pickle.dump(meta_data_dict, meta_file)
		meta_file.close()

	def sub_cb(self, sync_data):

		self.stop = sync_data.data[70]
	
		if self.time_step == 0:
			self.initial_xela_row = np.array(sync_data.data[:48])[np.newaxis, :]

		if self.stop == 0.0:
			self.slip_onset_vx = sync_data.data[-2]
			self.post_slip_lin_acc_T = sync_data.data[-1]
			self.state_vec = sync_data.data[64:70]
			self.marker_data[self.time_step, :] = sync_data.data[-9:-2]
			try:
				self.current_actions = self.robot_actions_pre_calc[self.time_step+10]
			except:
				self.current_actions = np.zeros(6)
			self.action_vec = [self.current_actions[0], self.current_actions[1], self.current_actions[2], self.current_actions[3], self.current_actions[4], self.current_actions[5]]
			self.xela_vec = np.array(sync_data.data[:48])[np.newaxis, :]
			self.xela_vec = self.remove_offset(self.xela_vec)

			self.kalman_filter_func()

			self.xela_data[self.time_step, : ] = np.concatenate((self.xela_vec[0], self.xela_vec[1], self.xela_vec[2]))
			self.robot_data[self.time_step, : ]  = self.state_vec
			self.action_data[self.time_step, : ] = self.action_vec
					
		self.time_step +=1
	
	def compute_trajectory(self):

		xela_seq   = self.xela_kalman_mean[self.time_step-10 : self.time_step , : ]
		robot_seq  = self.robot_data[self.time_step-10 : self.time_step , : ]
		action_seq = self.action_data[self.time_step-10 : self.time_step , : ] # shpe: 10x6
		self.scaled_tactile, self.scaled_action = self.scale_data(xela_seq, np.concatenate((robot_seq, action_seq), axis=0)) # shape 10x1x48 & 20x1x6

		# Predictor
		pred_ort_inputs = {self.pred_ort_session.get_inputs()[0].name: self.scaled_tactile, self.pred_ort_session.get_inputs()[1].name: self.scaled_action}
		self.prediction = self.pred_ort_session.run(None, pred_ort_inputs)
		# Classifier
		classifier_input = self.preprocess_predictions(self.prediction[0][:, 0, :])
		class_ort_inputs = {self.class_ort_session.get_inputs()[0].name: classifier_input}
		classification = self.class_ort_session.run(None, class_ort_inputs)
		slip_class = expit(classification[0])
		self.slip_pred_tag = np.round(slip_class)

		self.add_to_save()
		
		self.prev_time_step = self.time_step

	def control_loop(self):

		rate = rospy.Rate(59)
		
		while not rospy.is_shutdown():
			
			try:
				if self.time_step == 0:
					self.t0 = time.time()

				if self.stop == 0.0 and self.time_step > 10 and self.time_step > self.prev_time_step:
					self.compute_trajectory()
				elif self.stop == 1.0:
					[sub.sub.unregister() for sub in self.sync_subscriber]
					break

				rate.sleep()
			
			except KeyboardInterrupt:
				break
	
	def print_rate(self):
		t1 = time.time()
		rate = self.time_step / (t1-self.t0)
		print("RATE: ", rate)

if __name__ == "__main__":	
	mf = RobotTest()

	mf.print_rate()
	mf.save_results()