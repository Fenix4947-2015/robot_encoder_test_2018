package org.usfirst.frc.team4947.robot.subsystems;

import org.usfirst.frc.team4947.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {
	
	// Constants.
	private static final double PERCENT_OUTPUT = 0.5;
	
	// Members.
	private final WPI_TalonSRX leftMotor1;
	private final WPI_TalonSRX rightMotor1;

	public DriveTrain() {
		leftMotor1 = createLeftMotor1();
		rightMotor1 = createRightMotor1();
	}
	
	private static WPI_TalonSRX createLeftMotor1() {
		 WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR1_ADDRESS);
		 return motor;
	}
	
	private static WPI_TalonSRX createRightMotor1() {
		 WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR1_ADDRESS);
		 return motor;
	}

	public void initDefaultCommand() {
	}
	
	public void forward() {
		setPercentOutput(PERCENT_OUTPUT);
	}
	
	public void reverse() {
		setPercentOutput(-PERCENT_OUTPUT);
	}	
	
	public void stop() {
		setPercentOutput(0.0);
	}

	private void setPercentOutput(double percentOutput) {
		leftMotor1.set(ControlMode.PercentOutput, percentOutput);
		rightMotor1.set(ControlMode.PercentOutput, percentOutput);
	}
	
	public void dump() {
		SmartDashboard.putNumber("Left sensor velocity", leftMotor1.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Left sensor position", leftMotor1.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("Right sensor velocity", rightMotor1.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Right sensor position", rightMotor1.getSelectedSensorPosition(0));
	}
}