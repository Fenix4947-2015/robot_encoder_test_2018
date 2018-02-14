/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4947.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Joysticks.
	public static final int DRIVER_JOYSTICK_PORT = 0;
	
	// Drive train.
	public static final int LEFT_MOTOR1_ADDRESS = 3;
	public static final int RIGHT_MOTOR1_ADDRESS = 7;
}