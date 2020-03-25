#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hebi
import math
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

######################################################################
######################################################################
#####
#####     歩行器を押したら動いて引いたら止まる
#####
######################################################################
######################################################################




###################################################################
####   HEBIの初期設定など
###################################################################
lookup = hebi.Lookup()
#group = lookup.get_group_from_names(['Wheel'], ['Right', 'Left'])
group = lookup.get_group_from_names(['Wheel'], ['Right_light', 'Left_light'])

if group is None:
	print('Group not found: Did you forget to set the module family and names above?')
	exit(1)

for entry in lookup.entrylist:
	print(entry)

group_feedback = hebi.GroupFeedback(group.size)
group_command = hebi.GroupCommand(group.size)
group_info = hebi.GroupInfo(group.size)
group.feedback_frequency = 100

def Setting():
	group_command.reference_effort = 0.0000
	#group_command.reference_position = 0.0000

	new_velocity_kp_gains = [0.01, 0.01]
	new_effort_kp_gains = [0.1, 0.1]

	for new_gain in new_velocity_kp_gains:
		for i in range(group.size):
			group_command.velocity_kp = new_gain
		if not group.send_command_with_acknowledgement(group_command):
			print('Did not get acknowledgement from module when sending gains. Check connection.')
			exit(1)
		print('Set velocity kP gain to: {0}'.format(new_gain))

	for new_gain in new_effort_kp_gains:
		for i in range(group.size):
			group_command.effort_kp = new_gain
		if not group.send_command_with_acknowledgement(group_command):
			print('Did not get acknowledgement from module when sending gains. Check connection.')
			exit(1)
		print('Set effort kP gain to: {0}'.format(new_gain))


###########################################################################
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
		init_eff = group.get_next_feedback(reuse_fbk=group_feedback).effort
		self.init_pos = group_feedback.position
		sw = 0
		print("Please push if you want to move the walker\n")

		while not self.stop_event.is_set():
			
			group.get_next_feedback(reuse_fbk=group_feedback)
			velocity = self.velocity

			eff = group_feedback.effort
			difR = eff[0] - init_eff[0]
			difL = eff[1] - init_eff[1]

			# 歩行器押したら起動
			if sw == 0:
				if difL < -0.3 and difR > 0.3:
					print("Move##################################")
					sw = 1
					sleep(1)
					start = time()
					effortR = group_feedback.effort[0]
					effortL = group_feedback.effort[1]
			else:
				d = time() - start
				if sw == 1:
					print('Moving')
					velocityR = group_feedback.velocity[0]
					velocityL = group_feedback.velocity[1]

					if velocity == 0:
						effortR = 0.0
						effortL = 0.0
					else:
						if velocityR > -velocity: #加速
							effortR -= 0.01
						elif velocityR <= -velocity: #減速
							effortR += 0.01
						
						if velocityL < velocity: #加速
							effortL += 0.01
						elif velocityL >= velocity: #減速
							effortL -= 0.01

					group_command.effort = [effortR, effortL]
					group.send_command(group_command)

					current_R = group_feedback.motor_current[0]
					current_L = group_feedback.motor_current[1]

					if current_R > 1.0 or current_L > 1.0:
						for i in range(10):
							print('Stop{0}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'.format(i))
						sw = 2
						effortR = 0.0
						effortL = 0.0
						group_command.effort = [0.0, 0.0]
						group.send_command(group_command)

					"""
					# 色で判断できない
					led_R = group_feedback.led.color[0][0]
					#led_L = group_feedback.led[0]
					print(led_R)

					if led_R == 'red' or led_L == 'red':
						group_command.effort = [0.0, 0.0]
						group.send_command(group_command)
						self.stop_event.set()
					"""

				elif sw == 2:
					effortR = 0.0
					effortL = 0.0
					sleep(1)
					group_command.velocity = [0.0, 0.0]
					group.send_command(group_command)
					sleep(1)
					group_command.effort = [0.0, 0.0]
					group.send_command(group_command)
					sleep(4)
					sw = 3

				elif sw == 3:
					if difL < -0.3 and difR > 0.3:
						print("Move##################################")
						sw = 1
						sleep(1)

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

				self.position[0] = group_feedback.position[0]
				self.position[1] = group_feedback.position[1]



	def Move(self, velocity0):
		velocity = velocity0
		if velocity >= 8.0:
			velocity = 8.0
		self.velocity = velocity

	def MakeSomething(self):
		newpass = input("Create new folder: ")
		new_dir_path = "./" + newpass
		os.makedirs(new_dir_path, exist_ok=True)
		csv_name = input("Create new csv file: ")
		self.csv_name = csv_name
		self.new_dir_path = os.path.join(new_dir_path, csv_name + '.csv')


if __name__ == '__main__': # walker_effect_ver1.pyを実行したときだけ以下のものを実行

	Setting()
	th = MyThread()
	th.MakeSomething()
	th.start()

	key_velocity = None
	try:
		key_velocity = input("velocity: ")
	except TypeError as e:
		print('TypeError: ', e)
		key_velocity = '0.0'
	sleep(2)
	while True:
		try:
			key_velocity = input("velocity: ")
		except TypeError as e:
			print('TypeError: ', e)
			th.stop()
			break
		else:
			try:
				velocity0 = float(key_velocity)
			except ValueError as e:
				print('ValueError: ', e)
				th.stop()
				break
			else:
				if velocity0 >= 0:
					th.Move(velocity0)
				elif velocity0 < 0:
					print("TRY AGAIN")

	sleep(1)
	group_command.velocity = [0.0, 0.0]
	group.send_command(group_command)
	sleep(1)
	group_command.effort = [0.0, 0.0]
	group.send_command(group_command)
	print('####################################################')
	print('END')
	print('####################################################\n')
