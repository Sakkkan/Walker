import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import os

def Exe():
	plt.rcParams['font.family'] ='sans-serif'#使用するフォント
	plt.rcParams['xtick.direction'] = 'in'#x軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')
	plt.rcParams['ytick.direction'] = 'in'#y軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')
	plt.rcParams['xtick.major.width'] = 1.0#x軸主目盛り線の線幅
	plt.rcParams['ytick.major.width'] = 1.0#y軸主目盛り線の線幅
	#plt.rcParams['font.size'] = 14 #フォントの大きさ
	plt.rcParams['axes.linewidth'] = 1.0# 軸の線幅edge linewidth。囲みの太さ

	folder = input("Which folder: ")
	csv = input("csv: ")
	noise = input("Without noise? y or n?: ")
	if '.csv' in csv:
		csvfile = csv
	else:
		csvfile = csv + ".csv"
	csv_name = [
		'Time',
		'Velocity_Right',
		'Velocity_Left',
		'Effort_Right',
		'Effort_Left',
		'Velocity_Command_Right',
		'Velocity_Command_Left',
		'Effort_Command_Right',
		'Effort_Command_Left',
		'Position_Right',
		'Position_Left',
		'Motor_Position_Right',
		'Motor_Position_Left',
		'Motor_Velocity_Right',
		'Motor_Velocity_Left',
		'AccelerometerX_Right',
		'AccelerometerY_Right',
		'AccelerometerZ_Right',
		'AccelerometerX_Left',
		'AccelerometerY_Left',
		'AccelerometerZ_Left',
		'GyroX_Right',
		'GyroY_Right',
		'GyroZ_Right',
		'GyroX_Left',
		'GyroY_Left',
		'GyroZ_Left',
		'Current_Right',
		'Current_Left',
		'Voltage_Right',
		'Voltage_Left'
		]
	os.chdir("./" + folder)
	# csv 読み取り
	data = pd.read_csv(
		csvfile,
		names = csv_name
		)
	data_col = data.columns
	#data = data[:]

	fc = 5
	N = len(data)
	dt = 0.01 # HEBIの周波数変えたらここも変える##############################################################
	hammingWindow = np.hamming(N)    # ハミング窓

	# パラメータ設定
	fn = 1/(2*dt)                   # ナイキスト周波数
	#fp = 2                          # 通過域端周波数[Hz] fcと同じ
	fs = 15                         # 阻止域端周波数[Hz]
	gpass = 3                     # 通過域最大損失量[dB]
	gstop = 40                      # 阻止域最小減衰量[dB]
	norm_pass = fc/fn
	norm_stop = fs/fn
	# 正規化
	Wp = norm_pass
	Ws = norm_stop

	# バターワースフィルタ
	Nbutter, Wn = signal.buttord(Wp, Ws, gpass, gstop, analog=0)
	b1, a1 = signal.butter(Nbutter, Wn, btype="low", analog=0, output='ba')
	time = data[data_col[0]]
	if noise == 'n':
	#########################################################
	#####          ノイズ除去なし                       #####
	#########################################################
		vel_R = data[data_col[1]]
		vel_L = data[data_col[2]]
		eff_R = data[data_col[3]]
		eff_L = data[data_col[4]]
		velcom_R = data[data_col[5]]
		velcom_L = data[data_col[6]]
		effcom_R = data[data_col[7]]
		effcom_L = data[data_col[8]]
		pos_R = data[data_col[9]]
		pos_L = data[data_col[10]]
		Mpos_R = data[data_col[11]]
		Mpos_L = data[data_col[12]]
		Mvel_R = data[data_col[13]]
		Mvel_L = data[data_col[14]]
		accelX_R = data[data_col[15]]
		accelY_R = data[data_col[16]]
		accelZ_R = data[data_col[17]]
		accelX_L = data[data_col[18]]
		accelY_L = data[data_col[19]]
		accelZ_L = data[data_col[20]]
		gyroX_R = data[data_col[21]]
		gyroY_R = data[data_col[22]]
		gyroZ_R = data[data_col[23]]
		gyroX_L = data[data_col[24]]
		gyroY_L = data[data_col[25]]
		gyroZ_L = data[data_col[26]]
		current_R = data[data_col[27]]
		current_L = data[data_col[28]]
		voltage_R = data[data_col[29]]
		voltage_L = data[data_col[30]]

	elif noise == 'y':
	#########################################################
	#####          ノイズ除去あり                       #####
	#########################################################
		vel_R = signal.filtfilt(b1, a1, data[data_col[1]])
		vel_L = signal.filtfilt(b1, a1, data[data_col[2]])
		eff_R = signal.filtfilt(b1, a1, data[data_col[3]])
		eff_L = signal.filtfilt(b1, a1, data[data_col[4]])
		velcom_R = signal.filtfilt(b1, a1, data[data_col[5]])
		velcom_L = signal.filtfilt(b1, a1, data[data_col[6]])
		effcom_R = signal.filtfilt(b1, a1, data[data_col[7]])
		effcom_L = signal.filtfilt(b1, a1, data[data_col[8]])
		pos_R = signal.filtfilt(b1, a1, data[data_col[9]])
		pos_L = signal.filtfilt(b1, a1, data[data_col[10]])
		Mpos_R = signal.filtfilt(b1, a1, data[data_col[11]])
		Mpos_L = signal.filtfilt(b1, a1, data[data_col[12]])
		Mvel_R = signal.filtfilt(b1, a1, data[data_col[13]])
		Mvel_L = signal.filtfilt(b1, a1, data[data_col[14]])
		accelX_R = signal.filtfilt(b1, a1, data[data_col[15]])
		accelY_R = signal.filtfilt(b1, a1, data[data_col[16]])
		accelZ_R = signal.filtfilt(b1, a1, data[data_col[17]])
		accelX_L = signal.filtfilt(b1, a1, data[data_col[18]])
		accelY_L = signal.filtfilt(b1, a1, data[data_col[19]])
		accelZ_L = signal.filtfilt(b1, a1, data[data_col[20]])
		gyroX_R = signal.filtfilt(b1, a1, data[data_col[21]])
		gyroY_R = signal.filtfilt(b1, a1, data[data_col[22]])
		gyroZ_R = signal.filtfilt(b1, a1, data[data_col[23]])
		gyroX_L = signal.filtfilt(b1, a1, data[data_col[24]])
		gyroY_L = signal.filtfilt(b1, a1, data[data_col[25]])
		gyroZ_L = signal.filtfilt(b1, a1, data[data_col[26]])
		current_R = signal.filtfilt(b1, a1, data[data_col[27]])
		current_L = signal.filtfilt(b1, a1, data[data_col[28]])
		voltage_R = signal.filtfilt(b1, a1, data[data_col[29]])
		voltage_L = signal.filtfilt(b1, a1, data[data_col[30]])
	#########################################################
	#########################################################

	plot_list = [
		[-vel_R, vel_L],
		[-eff_R, eff_L],
		[-velcom_R, velcom_L],
		[-effcom_R, effcom_L],
		[-pos_R, pos_L],
		[-Mpos_R, Mpos_L],
		[-Mvel_R, Mvel_L],
		[accelX_R, accelY_R, accelZ_R, accelX_L, accelY_L, accelZ_L],
		[gyroX_R, gyroY_R, gyroZ_R, gyroX_L, gyroY_L, gyroZ_L],
		[current_R, current_L],
		[voltage_R, voltage_L]
	]

	ylabel_list = [
		['Velocity_Right[rad/s]', 'Velocity_Left[rad/s]'],
		['Effort_Right[Nm]', 'Effort_Left[Nm]'],
		['Velocity_Command_Right[rad/s]', 'Velocity_Command_Left[rad/s]'],
		['Effort_Command_Right[Nm]', 'Effort_Command_Left[Nm]'],
		['Position_Right[rad]', 'Position_Left[rad]'],
		['Motor_Position_Right[rad]', 'Motor_Position_Left[rad]'],
		['Motor_Velocity_Right[rad/s]', 'Motor_Velocity_Left[rad/s]'],
		['Accelerometer_Right', 'Accelerometer_Left'],
		['Gyro_Right', 'Gyro_Left'],
		['Current_Right', 'Current_Left'],
		['Voltage_Right', 'Voltage_Left']
	]

	figtitle_list = [
		'velocity' + '_' + csv,
		'effort' + '_' + csv,
		'velocity_command' + '_' + csv,
		'effort_command' + '_' + csv,
		'position' + '_' + csv,
		'motor_position' + '_' + csv,
		'motor_velocity' + '_' + csv,
		'accelerometer' + '_' + csv,
		'gyro' + '_' + csv,
		'current' + '_' + csv,
		'voltage' + '_' + csv
	]
	figtitle_len = len(figtitle_list)

	print(data.describe())
	###############################################################################################
	for i in range(figtitle_len):
		if 'accelerometer' in str(figtitle_list[i]) or 'gyro' in str(figtitle_list[i]):
			plt.figure(dpi=300)
			#plt.subplot(211)
			plt.plot(time, plot_list[i][0], color='r')
			plt.plot(time, plot_list[i][1], color='g')
			plt.plot(time, plot_list[i][2], color='b')
			plt.xlabel('Time[sec]')
			plt.ylabel(ylabel_list[i][0])
			#plt.xticks( np.arange(5.0, 21.0, 5.0) )
			#plt.yticks( np.arange(-0.5, 0.6, 0.25) )
			plt.savefig(figtitle_list[i] + '_R.png')
			print(figtitle_list[i] + ' RIght Complete')
			plt.close()

			plt.figure(dpi=300)
			#plt.subplot(212)
			plt.plot(time, plot_list[i][3], color='r')
			plt.plot(time, plot_list[i][4], color='g')
			plt.plot(time, plot_list[i][5], color='b')
			plt.xlabel('Time[sec]')
			plt.ylabel(ylabel_list[i][1])
			#plt.xticks( np.arange(5.0, 21.0, 5.0) )
			#plt.yticks( np.arange(-0.5, 0.6, 0.25) )
			plt.savefig(figtitle_list[i] + '_L.png')
			print(figtitle_list[i] + ' Left Complete')
			plt.close()

		else:
			plt.figure(dpi=300)
			#plt.subplot(211)
			plt.plot(time, plot_list[i][0], color='r')
			plt.xlabel('Time[sec]')
			plt.ylabel(ylabel_list[i][0])
			#plt.xticks( np.arange(5.0, 21.0, 5.0) )
			#plt.yticks( np.arange(-0.5, 0.6, 0.25) )
			plt.savefig(figtitle_list[i] + '_R.png')
			print(figtitle_list[i] + ' Right Complete')
			plt.close()

			plt.figure(dpi=300)
			#plt.subplot(212)
			plt.plot(time, plot_list[i][1], color='b')
			plt.xlabel('Time[sec]')
			plt.ylabel(ylabel_list[i][1])
			#plt.xticks( np.arange(5.0, 21.0, 5.0) )
			#plt.yticks( np.arange(-0.5, 0.6, 0.25) )
			plt.savefig(figtitle_list[i] + '_L.png')
			print(figtitle_list[i] + ' Left Complete')
			plt.close()
	###############################################################################################


	freq = np.linspace(0, 1.0/dt, N)
	windowData = hammingWindow * plot_list[1][0]
	F = np.fft.fft(data[data_col[3]])
	F_win = np.fft.fft(windowData)
	F_win = F_win/max(abs(F_win))
	plt.figure(dpi=300)
	plt.plot(freq,np.abs(F_win), label = "Right", color='r')
	plt.xlabel('Frequency [Hz]')
	plt.ylabel('Power')
	plt.legend()
	plt.xlim(0.0, 10.0)
	plt.xticks( np.arange(0.0, 10.0, 1.0) )
	plt.savefig(figtitle_list[1] + '_R_fft.png')
	print('\n' + figtitle_list[1] + ' fft Right Complete')
	plt.close()

	windowData = hammingWindow * plot_list[1][1]
	F = np.fft.fft(data[data_col[4]])
	F_win = np.fft.fft(windowData)
	F_win = F_win/max(abs(F_win))
	plt.figure(dpi=300)
	plt.plot(freq,np.abs(F_win), label = "Left", color='b')
	plt.xlabel('Frequency [Hz]')
	plt.ylabel('Power')
	plt.legend()
	plt.xlim(0.0, 10.0)
	plt.xticks( np.arange(0.0, 10.0, 1.0) )
	plt.savefig(figtitle_list[1] + '_L_fft.png')
	print(figtitle_list[1] + ' fft Left Complete')
	plt.close()

	#plt.show()

if __name__ == '__main__':
	Exe()