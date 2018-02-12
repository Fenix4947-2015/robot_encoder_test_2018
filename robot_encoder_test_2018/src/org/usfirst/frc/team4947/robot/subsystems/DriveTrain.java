package org.usfirst.frc.team4947.robot.subsystems;

import org.usfirst.frc.team4947.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	
	// Constants.
	private static final double PERCENT_OUTPUT = 0.5;
	
	private static final double INCHES_PER_FOOT = 12.0;

	private static final double ENCODER_SHAFT_DIAMETER_IN_INCHES = 0.25;
	private static final double ENCODER_SHAFT_RADIUS_IN_INCHES = (ENCODER_SHAFT_DIAMETER_IN_INCHES / 2);
	private static final double ENCODER_SHAFT_CIRCUMFERENCE_IN_INCHES = (2 * Math.PI * ENCODER_SHAFT_RADIUS_IN_INCHES);
	private static final double ENCODER_PULSES_PER_REVOLUTION = 1440;	
	
	// Members.
	private final WPI_TalonSRX leftMotor1;
	private final WPI_TalonSRX rightMotor1;

	public DriveTrain() {
		leftMotor1 = createLeftMotor1();
		rightMotor1 = createRightMotor1();
	}
	
	private static WPI_TalonSRX createLeftMotor1() {
		 WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR1_ADDRESS);
		 initEncoder(motor);
		 return motor;
	}
	
	private static WPI_TalonSRX createRightMotor1() {
		 WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR1_ADDRESS);
		 initEncoder(motor);
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
	
	// GitHub: https://github.com/CrossTheRoadElec/Phoenix-Documentation
	// Closed-loop: https://github.com/CrossTheRoadElec/Phoenix-Documentation#closed-loop-using-sensor-control
	// Source: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/PositionClosedLoop
	private static void initEncoder(WPI_TalonSRX motor) {
		// Choose the sensor and sensor direction.
		motor.configSelectedFeedbackSensor(
				FeedbackDevice.QuadEncoder, 
				DriveTrainConstants.kPIDLoopIdx,
				DriveTrainConstants.kTimeoutMs);

		// Choose to ensure sensor is positive when output is positive.
		motor.setSensorPhase(DriveTrainConstants.kSensorPhase);

		// Choose based on what direction you want forward/positive to be.
		// This does not affect sensor phase.
		motor.setInverted(DriveTrainConstants.kMotorInvert);

		// Set the peak and nominal outputs, 12V means full.
		motor.configNominalOutputForward(0, DriveTrainConstants.kTimeoutMs);
		motor.configNominalOutputReverse(0, DriveTrainConstants.kTimeoutMs);
		motor.configPeakOutputForward(PERCENT_OUTPUT, DriveTrainConstants.kTimeoutMs);
		motor.configPeakOutputReverse(-PERCENT_OUTPUT, DriveTrainConstants.kTimeoutMs);

		// Set the allowable closed-loop error, Closed-Loop output will be neutral within this range.
		// See Table in Section 17.2.1 for native units per rotation.
		motor.configAllowableClosedloopError(0, DriveTrainConstants.kPIDLoopIdx, DriveTrainConstants.kTimeoutMs);

		// Set closed loop gains in slot0, typically kF stays zero.
		motor.config_kF(DriveTrainConstants.kPIDLoopIdx, 0.0, DriveTrainConstants.kTimeoutMs);
		motor.config_kP(DriveTrainConstants.kPIDLoopIdx, 0.1, DriveTrainConstants.kTimeoutMs);
		motor.config_kI(DriveTrainConstants.kPIDLoopIdx, 0.0, DriveTrainConstants.kTimeoutMs);
		motor.config_kD(DriveTrainConstants.kPIDLoopIdx, 0.0, DriveTrainConstants.kTimeoutMs);

		// Lets grab the 360 degree position of the MagEncoder's absolute position, and intitally set the relative
		// sensor to match.
		int absolutePosition = motor.getSensorCollection().getPulseWidthPosition();
		
		// Mask out overflows, keep bottom 12 bits.
		absolutePosition &= 0xFFF;
		
		if (DriveTrainConstants.kSensorPhase) {
			absolutePosition *= -1;
		}
		
		if (DriveTrainConstants.kMotorInvert) {
			absolutePosition *= -1;
		}

		// Set the quadrature (relative) sensor to match absolute.
		motor.setSelectedSensorPosition(
				absolutePosition, 
				DriveTrainConstants.kPIDLoopIdx,
				DriveTrainConstants.kTimeoutMs);
	}

	public void forward(double feet) {
		double inches = (feet * INCHES_PER_FOOT);

		forward(leftMotor1, inches);
		forward(rightMotor1, inches);
	}

	private void forward(WPI_TalonSRX motor, double inches) {
		double rotations = (inches / ENCODER_SHAFT_CIRCUMFERENCE_IN_INCHES);
		double targetPositionRotations = (rotations * ENCODER_PULSES_PER_REVOLUTION);

		motor.set(ControlMode.Position, targetPositionRotations);
	}
}