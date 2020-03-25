#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import shutil

def Delete():
	yesno = input("Delete Folder? y or n: ")
	if 'y' in yesno:
		folder = input("folder name: ")
		shutil.rmtree('./' + folder)
	else:
		yesno1 = input("Delete File? y or n: ")
		if 'y' in yesno1:
			folder1 = input("Folder where you want to delete FILE: ")
			file1 = input("file name: ")
			os.remove('./' + folder1 + '/' + file1)


if __name__ == '__main__':

	Delete()
	print('####################################################')
	print('END')
	print('####################################################\n')