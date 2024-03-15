Author : 	Haysen Hexiang Wang
College: 	Homerton
CRSid: 		hw560

Project: Determining whether the wearer is standing upright at about 90 deg, or lying down at about 0 deg. 
In the driver file of accelerometer MMA8451Q, there is a orientation determination function called: orientation() in devMMA8451Q.c

## devMMA8451Q.c
where the decision boundary is defined: 
		if sum_y >= 200000,
			The board is 90 degree upright, so the wearer is standing upright.
		if sum_z >= 200000,
			The board is 0 degree flat, so the wearer is lying down.

A function void set_0x2a() is added, to set the accelerometer to active mode from standby mode. Plus a high resolution mode. 

## Boot.c
This implementation is in the WARP menu at line 2758, by pressing option 'y': "reading from MMA8451Q". 
		if 0 is pressed,
			The WARP will return real time accelerometer readings at x, y, z axis correspondingly.
		if 1 is pressed,
			The WARP will return the state instead: standing upright or lying down. 	

## config.h
Set the MMA8451Q to 1.