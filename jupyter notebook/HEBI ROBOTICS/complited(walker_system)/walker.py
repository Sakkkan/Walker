#!/usr/bin/env python
# -*- coding: utf-8 -*-

#================================================
#   歩行器をキーボード入力で回転速度を可変にした
#================================================

import hebi
import math
from math import pi
import scipy.constants
import numpy as np
import threading
#from getch import getch, pause
from time import sleep, time
import csv
import os

lookup = hebi.Lookup()
#group = lookup.get_group_from_names(['Wheel', 'HEBI'], ['Right', 'Left', 'IO_BOARD'])
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

group.feedback_frequency = 50
group_command.reference_effort = 0.0000

new_velocity_kp_gains = [0.01, 0.01]

for new_gain in new_velocity_kp_gains:
	for i in range(group.size):
		group_command.velocity_kp = new_gain
	if not group.send_command_with_acknowledgement(group_command):
		print('Did not get acknowledgement from module when sending gains. Check connection.')
		exit(1)
	print('Set velocity kP gain to: {0}'.format(new_gain))

class MyThread(threading.Thread):
	
	def __init__(self):
		
		super(MyThread, self).__init__()
		self.com = np.zeros(2)
		self.stop_event = threading.Event()
		self.setDaemon(True)
		self.effort = 0.0
		
		
	def stop(self):
		
		self.stop_event.set()

		
	def run(self):
		print("=========================================\n     w     \n  a  s  d  \n     x     \n=========================================\n\nIf you want to quit, please press 'q'.\n")
		
		start = time()
		global csv_name
		csv_name = ''
		#init_eff = group.get_next_feedback().effort
		while not self.stop_event.is_set():
			
			d = time() - start
			#while d <= 2.0:
			group.get_next_feedback(reuse_fbk=group_feedback)
			x = self.com[0]
			y = self.com[1]

			spring_constant = x # rad/sec
			group_command.velocity = [spring_constant * y, -spring_constant * y]
			
			#group_command.effort = [-1.0, 1.0]
			group.send_command(group_command)


			"""
			eff = group_feedback.effort
			dif_E = np.average(eff - init_eff)

			vel = group_feedback.velocity[1]
			vel_com = group_feedback.velocity_command[1]
			if vel_com != 0:
				dif_V = vel / vel_com
			else:
				dif_V = 0.0
			"""

			#d = time() - start
			if d <= 15:
				with open(csv_name + '.csv','a', newline='') as f:
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
						group_feedback.gyro[1][2]
						]
					writer.writerow(outputdata)
					#writer.writerow([dif_E, vel_com, vel, dif_V])
			#print("{0},{1},{2}".format(d,eff[0],eff[1]))
			#print("{0:+.3f},{1:+.3f},{2:+.3f},{3:+.3f}".format(dif_E, vel_com, vel, dif_V))
			#print("dif_E: {0:+.3f}, vel_com: {1:+.3f}, vel: {2:+.3f}, dif_V: {3:+.3f}".format(dif_E, vel_com, vel, dif_V))
		
		
		
		
		
	def Move(self, key):
		
		"""
		key = key
		x = 0.1
		y = 0.5
		com = self.com
		
		if key == 119: # w 前進
			com[0] += x
			com[1] = 1.0
			com[2] = 1.0
		elif key == 113: # q 左折
			com[1] = y
			com[2] = 1.0
		elif key == 101: # e 右折
			com[1] = 1.0
			com[2] = y
		elif key ==115: # s 停止
			while not com[0]==0:
				if com[0] >= 0.1:
					com[0] -= x
				elif com[0] <= -0.1:
					com[0] += x
				else:
					com[0] = 0.0
			com[1] = 0.0
			com[2] = 0.0
		elif key == 97: # a 左旋回
			if com[0] >= 0.0:
				com[1] = -1.0
				com[2] = 1.0
			elif com[0] < -0.0:
				com[1] = 1.0
				com[2] = -1.0
		elif key == 100: # d 右旋回
			if com[0] >= 0.0:
				com[1] = 1.0
				com[2] = -1.0
			elif com[0] < -0.0:
				com[1] = -1.0
				com[2] = 1.0
		elif key == 120: # x 後退
			com[0] -= x
			com[1] = 1.0
			com[2] = 1.0
		
		if com[0] > pi:
			com[0] = 3.0
		elif com[0] < -pi:
			com[0] = -3.0
			
		
		print('x = {0:.2f}, yL = {1:.2f}, yR = {2:.2f}'.format(com[0], com[1], com[2]))
		"""

		key = key
		#x = 1.0
		y = 0.5
		com = self.com
		
		
		if key == 'a': #97: # a 左折
		#elif key == 97:
			com[1] = y
		elif key == 'd': #100: # d 右折
		#elif key == 100:
			com[1] = -y
		elif key == 's': #115: # s 停止
		#elif key == 115:
			com[0] = 0.0
			com[1] = 0.0
		else:
			com[0] = float(key)
			com[1] = 1.0
		
		"""
		if com[0] > 3.0:
			com[0] = 3.0
		elif com[0] < -3.0:
			com[0] = -3.0
		"""

		self._com = com
		
		print("{0:.2f}, {1:.2f}".format(com[0], com[1]))
		
if __name__ == '__main__':
	th = MyThread()
	th.start()
	
	"""
	if os.path.exists('effort.csv'):
		os.remove('effort.csv')
	else:
		print("No exist")
	"""

	global csv_name
	csv_name = input("Crate name of csv file:")
	key = None
	while True:
		if key == None:
			key = input("Speed: ")
			sleep(2)
		else:
			key = input("Speed: ")
		#key = ord(getch())
		if 'q' in key: # q
		#if key == 113:
			th.stop()
			break
		else:
			#effort = float(input("Effort: "))
			#if effort > 0.0 and effort <= 25.0:
			th.Move(key)
			
	sleep(1)
	#group_command = hebi.GroupCommand(group.size)
	group_command.velocity = [0.0, 0.0]
	#group_command.effort = [0.0, 0.0]
	group.send_command(group_command)
	print('END')
