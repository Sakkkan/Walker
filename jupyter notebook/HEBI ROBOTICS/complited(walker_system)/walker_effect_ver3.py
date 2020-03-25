#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hebi
from math import pi
import scipy.constants
import numpy as np
import threading
from time import sleep, time
import csv
import os
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import datetime
import serial

######################################################################
######################################################################
#####
#####     歩行器を一定に走行する（M5Stackあり）
#####
######################################################################
######################################################################



###################################################################
####   HEBIの初期設定など
###################################################################
lookup = hebi.Lookup()
#group = lookup.get_group_from_names(['Wheel'], ['Right', 'Left'])
group = lookup.get_group_from_names(['Wheel'], ['Right_light', 'Left_light'])
no_assist = False

if group is None:
	print('Group not found: Did you forget to set the module family and names above?')

for entry in lookup.entrylist:
	print(entry)

try:
	group_feedback = hebi.GroupFeedback(group.size)
	group_command = hebi.GroupCommand(group.size)
	group_info = hebi.GroupInfo(group.size)
	group.feedback_frequency = 100
except:
	no_assist = True

def Setting():
	group_command.reference_effort = [0,0]

	new_velocity_kp_gains = [0.01, 0.01]
	new_effort_kp_gains = [0.1, 0.1]

	for new_gain in new_velocity_kp_gains:
		for i in range(group.size):
			group_command.velocity_kp = new_gain
		if not group.send_command_with_acknowledgement(group_command):
			print('Did not get acknowledgement from module when sending gains. Check connection.')
			exit(1)
		#print('Set velocity kP gain to: {0}'.format(new_gain))

	for new_gain in new_effort_kp_gains:
		for i in range(group.size):
			group_command.effort_kp = new_gain
		if not group.send_command_with_acknowledgement(group_command):
			print('Did not get acknowledgement from module when sending gains. Check connection.')
			exit(1)
		#print('Set effort kP gain to: {0}'.format(new_gain))


###################################################################
####   HEBIの制御系
###################################################################
class MyThread(threading.Thread):
	
	def __init__(self):

		super(MyThread, self).__init__()
		self.velocity = 0.0
		self.stop_event = threading.Event()
		self.setDaemon(True)
		self.csv_name = ''
		self.new_dir_path = None
		self.position = np.zeros(2)
		self.init_pos = np.zeros(2)
		
	def stop(self):
		self.stop_event.set()

		positionR = self.position[0] * -1
		positionL = self.position[1]
		R = 150 # 車輪の直径
		distance = R / 1000 / 4 * (positionR + positionL - self.init_pos[0] - self.init_pos[1])
		print('走行距離[m]: ', distance)

	def run(self):

		csvfile = self.new_dir_path
		group.get_next_feedback(reuse_fbk=group_feedback)
		self.init_pos = group_feedback.position
		sw = 0
		print0 = True

		while not self.stop_event.is_set():
			
			group.get_next_feedback(reuse_fbk=group_feedback)
			velocity = self.velocity

			if sw == 0: # 初期状態、歩行器動かない
				if velocity == 0:
					if print0 == True:
						print("State0")
						print0 = False
						print1 = True
						print2 = True
						print3 = True
				else:
					group_command.position = [np.nan, np.nan] # positionコマンドが保持されているから消す
					group.send_command(group_command)
					print("sw 0 -> 1")
					sw = 1
					sw1 = 1.1
					start = time()
					effortR = 0
					effortL = 0
			else:
				d = time() - start
				if sw == 1: # 歩行器が動く
					velocityR = group_feedback.velocity[0]
					velocityL = group_feedback.velocity[1]

					if sw1 == 1.1:
						if print1 == True:
							print("Accelerator==========================================")
							print1 = False
						if (velocityR <= -velocity) or (velocityL >= velocity):
							sw1 = 1.2
						else:
							effortR -= 0.02 #加速
							effortL += 0.02 #加速

					elif sw1 == 1.2:
						if print2 == True:
							print("Keep Speed-------------------------------------------")
							print2 = False

						if velocity == 0:
							sw1 = 1.3
						else:
							if velocityR > -velocity: #加速
								effortR -= 0.01
							elif velocityR <= -velocity: #減速
								effortR += 0.01
							
							if velocityL < velocity: #加速
								effortL += 0.01
							elif velocityL >= velocity: #減速
								effortL -= 0.01

					elif sw1 == 1.3:
						if print3 == True:
							print("Down Speed###########################################")
							print3 = False

						if velocityR >= -0.2 or velocityL <= 0.2:
							sw = 2
						else:
							effortR += 0.02 #減速
							effortL -= 0.02 #減速

					group_command.effort = [effortR, effortL]
					group.send_command(group_command)
					current_R = group_feedback.motor_current[0]
					current_L = group_feedback.motor_current[1]

					if current_R > 1.2 or current_L > 1.2:
						for i in range(10):
							print('Stop{0}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'.format(i))
						sw = 2

				elif sw == 2: # 歩行器を「安全に」完全停止させるための対策
					effortR = 0.0
					effortL = 0.0
					group_command.effort = [0.0, 0.0]
					group.send_command(group_command)
					sleep(0.5)
					posR = group_feedback.position[0]
					posL = group_feedback.position[1]
					group_command.position = [posR, posL]
					group_command.effort = [np.nan, np.nan] # effortコマンドが保持されているから消す
					group.send_command(group_command)
					sw = 3
					print("State2")
					print("sw 2 -> 3")
					print4 = True

				elif sw == 3: # 歩行器をいつでも動かせる状態
					if velocity == 0:
						if print4 == True:
							print("State3")
							print4 = False
					else:
						print("sw 3 -> 1")
						sw = 1
						sw1 = 1.1
						effortR = 0
						effortL = 0
						print1 = True
						print2 = True
						print3 = True
						group_command.position = [np.nan, np.nan] # positionコマンドが保持されているから消す
						group.send_command(group_command)

				# アクチュエータにあるデータをとりあえずすべて保存
				# [0] -> Right, [1] -> Left, [][0] -> x, [][1] -> y, [][2] -> z
				with open(csvfile,'a', newline='') as f:
					writer = csv.writer(f)
					outputdata = [
						d,
						group_feedback.velocity[0],
						group_feedback.velocity[1],
						group_feedback.effort[0],
						group_feedback.effort[1],
						group_feedback.velocity_command[0],
						group_feedback.velocity_command[1],
						group_feedback.effort_command[0],
						group_feedback.effort_command[1],
						group_feedback.position[0],
						group_feedback.position[1],
						group_feedback.motor_position[0],
						group_feedback.motor_position[1],
						group_feedback.motor_velocity[0],
						group_feedback.motor_velocity[1],
						group_feedback.accelerometer[0][0],
						group_feedback.accelerometer[0][1],
						group_feedback.accelerometer[0][2],
						group_feedback.accelerometer[1][0],
						group_feedback.accelerometer[1][1],
						group_feedback.accelerometer[1][2],
						group_feedback.gyro[0][0],
						group_feedback.gyro[0][1],
						group_feedback.gyro[0][2],
						group_feedback.gyro[1][0],
						group_feedback.gyro[1][1],
						group_feedback.gyro[1][2],
						group_feedback.motor_current[0],
						group_feedback.motor_current[1],
						group_feedback.voltage[0],
						group_feedback.voltage[1]
						]
					writer.writerow(outputdata)

				# 走行距離計算に使用
				self.position[0] = group_feedback.position[0]
				self.position[1] = group_feedback.position[1]
	
	# 歩行器の速度をアクチュエータの回転速度に変換
	def Move(self, velocity0):
		velocity = round(velocity0 / 0.15, 2)
		if velocity >= 9.42:
			print("It's over")
			velocity = 9.42
		self.velocity = velocity
	
	# フォルダー作成とcsvファイルの名前設定
	def MakeSomething(self):
		newpass = input("Create new folder: ")
		new_dir_path = "./" + newpass
		os.makedirs(new_dir_path, exist_ok=True)
		csv_name = input("Create new csv file: ")
		self.csv_name = csv_name
		self.new_dir_path = os.path.join(new_dir_path, csv_name + '.csv')

		global log_pass
		log_pass = os.path.join(new_dir_path, csv_name)


###################################################################
####   Arduinoで計測したものを受信しcsvで保存
###################################################################
class IMUBTRecorder(threading.Thread):
	
	def __init__(self, port, baudrate):

		super(IMUBTRecorder, self).__init__()
		self.stop_event = threading.Event()
		self.setDaemon(True)
		self._port = port
		self._baudrate = baudrate
		self._columns=['time','ax', 'ay', 'az', 'roll', 'pitch', 'yaw']
		self._data_frame = pd.DataFrame(columns=self._columns)
		self._start_at = None

	def _on_data_recieved(self, data):
		self._make_log(data)

	def _make_log(self, data):
		s = pd.Series(data)
		self._data_frame = self._data_frame.append(s, ignore_index=True)

	def _save(self,directory):
		self._data_frame.to_csv(directory.format(datetime.datetime.now()))
		
	def stop(self):
		global log_pass
		self.stop_event.set()
		print("Complete Recording")
		self._save(log_pass + '_log.csv')

	def run(self):

		parsed_data = []
		with serial.Serial(self._port, timeout=None, baudrate=self._baudrate) as se:
			print("Connected************************************************")

			while not self.stop_event.is_set():
				line = se.readline()
				# remove return and line feed
				str_data = str(line).split('\\r\\n')[:-1]
				
				if not str_data:
					continue
				# split by tag
				data = str_data[0].split('\\t')
				if len(data) < 7:
					continue
				# remove data name column
				data = data[1:]
				now = time()
				if not self._start_at:
					self._start_at = now
				parsed_data = {k: float(v) for k, v in zip(self._columns, data)}
				self._on_data_recieved(parsed_data)
				#print("ax:"+str(parsed_data['ax'])+"ay:"+str(parsed_data['ay'])+"az:"+str(parsed_data['az']))

		se.close()

if __name__ == '__main__':

	COM = '/dev/rfcomm0'

	if no_assist == False:
		# 起動時に歩行器を停止できる
		#group.get_next_feedback(reuse_fbk=group_feedback)
		#posR = group_feedback.position[0]
		#posL = group_feedback.position[1]
		#group_command.position = [posR, posL]

		Setting()
	
	th1 = MyThread()
	th1.MakeSomething()
	th2 = IMUBTRecorder(COM, 115200)
	th2.start()
	th1.start()
	key_velocity = None
	try:
		key_velocity = input("velocity: ")
	except TypeError as e:
		#print('TypeError: ', e)
		key_velocity = '0.0'
	except KeyboardInterrupt as e:
		th1.stop()
		th2.stop()
	sleep(2)
	while True:
		try:
			key_velocity = input("velocity: ")
		except TypeError as e:
			#print('TypeError: ', e)
			th1.stop()
			th2.stop()
			break
		except KeyboardInterrupt as e:
			th1.stop()
			th2.stop()
			break
		else:
			try:
				velocity0 = float(key_velocity)
			except ValueError as e:
				#print('ValueError: ', e)
				th1.stop()
				th2.stop()
				break
			except KeyboardInterrupt as e:
				th1.stop()
				th2.stop()
				break
			else:
				if velocity0 >= 0:
					th1.Move(velocity0)
				elif velocity0 < 0:
					print("TRY AGAIN")

	if no_assist == False:
		group_command.effort = [0.0, 0.0]
		group.send_command(group_command)
		sleep(0.5)
		posR = group_feedback.position[0]
		posL = group_feedback.position[1]
		group_command.effort = [np.nan, np.nan] # effortコマンドが保持されているから消す
		group_command.position = [posR, posL]
		group.send_command(group_command)

	print('####################################################')
	print('END')
	print('####################################################')
