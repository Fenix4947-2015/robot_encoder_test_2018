/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4947.robot.commands;

import org.usfirst.frc.team4947.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveDistance extends Command {
	
	// Constants.
	private static final double DISTANCE_FEET = 2.0;

	public DriveDistance() {
		requires(Robot.driveTrain);
	}

	@Override
	protected void initialize() {
		Robot.driveTrain.forward(DISTANCE_FEET);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected void end() {
		Robot.driveTrain.stop();
	}
}