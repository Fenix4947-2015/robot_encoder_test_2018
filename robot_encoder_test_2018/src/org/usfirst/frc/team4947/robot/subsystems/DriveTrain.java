package org.usfirst.frc.team4947.robot.subsystems;

import org.usfirst.frc.team4947.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {

	// Constants.
	private static final double PERCENT_OUTPUT = 0.6;

	private static final double WHEEL_DIAMETER_IN_INCHES = 6;
	private static final double WHEEL_RADIUS_IN_INCHES = (WHEEL_DIAMETER_IN_INCHES / 2.0);
	private static final double WHEEL_CIRCUMFERENCE_IN_INCHES = (2 * Math.PI * WHEEL_RADIUS_IN_INCHES);

	private static final double PULSES_PER_FOOT = 292;

	/** Not used. */
	private static final double INCHES_PER_FOOT = 12.0;
	private static final double ENCODER_PULSES_PER_REVOLUTION = 1440;
	private static final double INCHES_PER_ENCODER_REVOLUTION = (3 * WHEEL_CIRCUMFERENCE_IN_INCHES);

	// Members.
	private final WPI_TalonSRX leftMotor1;
	private final WPI_TalonSRX rightMotor1;

	public DriveTrain() {
		leftMotor1 = createLeftMotor1();
		rightMotor1 = createRightMotor1();
	}

	private static WPI_TalonSRX createLeftMotor1() {
		boolean kMotorInvert = false;

		WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.LEFT_MOTOR1_ADDRESS);
		initEncoder(motor, kMotorInvert);
		return motor;
	}

	private static WPI_TalonSRX createRightMotor1() {
		boolean kMotorInvert = true;

		WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR1_ADDRESS);
		initEncoder(motor, kMotorInvert);
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
	private static void initEncoder(WPI_TalonSRX motor, boolean kMotorInvert) {
		// Choose the sensor and sensor direction.
		motor.configSelectedFeedbackSensor(
				FeedbackDevice.QuadEncoder, 
				DriveTrainConstants.kPIDLoopIdx,
				DriveTrainConstants.kTimeoutMs);

		// Choose to ensure sensor is positive when output is positive.
		motor.setSensorPhase(DriveTrainConstants.kSensorPhase);

		// Choose based on what direction you want forward/positive to be.
		// This does not affect sensor phase.
		motor.setInverted(kMotorInvert);

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
		motor.config_kP(DriveTrainConstants.kPIDLoopIdx, 0.18, DriveTrainConstants.kTimeoutMs);
		motor.config_kI(DriveTrainConstants.kPIDLoopIdx, 0.0, DriveTrainConstants.kTimeoutMs);
		motor.config_kD(DriveTrainConstants.kPIDLoopIdx, 0.0, DriveTrainConstants.kTimeoutMs);

		int absolutePosition = 0;
		motor.setSelectedSensorPosition(
				absolutePosition, 
				DriveTrainConstants.kPIDLoopIdx,
				DriveTrainConstants.kTimeoutMs);
	}

	public void dump() {
		int leftPosition = leftMotor1.getSelectedSensorPosition(0);
		int rightPosition = rightMotor1.getSelectedSensorPosition(0);

		System.out.format("leftPosition=%d, rightPosition=%d%n", leftPosition, rightPosition);

		SmartDashboard.putNumber("Left Sensor position", leftPosition);
		SmartDashboard.putNumber("Right Sensor position", rightPosition);
	}

	public void forward(double feet) {
		forward(leftMotor1, feet);
		forward(rightMotor1, feet);
	}

	private void forward(WPI_TalonSRX motor, double feet) {
		// Used only for debugging.
		int pulseWidthPosition = motor.getSensorCollection().getPulseWidthPosition();
		int leftPosition = leftMotor1.getSelectedSensorPosition(0);
		int rightPosition = rightMotor1.getSelectedSensorPosition(0);		

		int absolutePosition = 0;
		motor.setSelectedSensorPosition(
				absolutePosition, 
				DriveTrainConstants.kPIDLoopIdx,
				DriveTrainConstants.kTimeoutMs);
		
		double targetPositionRotations = (feet * PULSES_PER_FOOT);
		
		System.out.format(
				"feet=%f, targetPositionRotations=%f, pulseWidthPosition=%d, leftPosition=%d, rightPosition=%d%n", 
				feet,
				targetPositionRotations, 
				pulseWidthPosition, 
				leftPosition, 
				rightPosition);
		
		motor.set(ControlMode.Position, targetPositionRotations);
	}
}