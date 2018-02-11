/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4947.robot;

import org.usfirst.frc.team4947.robot.commands.DriveForward;
import org.usfirst.frc.team4947.robot.commands.DriveReverse;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

	public enum XBoxAxis {
		LeftStickX(0), LeftStickY(1), LeftTrigger(2), RightTrigger(3), RightStickX(4), RightStickY(5);

		private final int value;

		XBoxAxis(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum XBoxButton {
		A(1), B(2), X(3), Y(4), LB(5), RB(6), Back(7), Start(8), LeftStick(9), RightStick(10);

		private final int value;

		XBoxButton(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	private final Joystick driverJoystick;

	public OI() {
		driverJoystick = createDriverJoystick();
	}

	@SuppressWarnings("unused")
	private static Joystick createDriverJoystick() {
		Joystick joystick = new Joystick(RobotMap.DRIVER_JOYSTICK_PORT);
		
        JoystickButton driverA = new JoystickButton(joystick, XBoxButton.A.getValue());
        JoystickButton driverB = new JoystickButton(joystick, XBoxButton.B.getValue());
        JoystickButton driverX = new JoystickButton(joystick, XBoxButton.X.getValue());
        JoystickButton driverY = new JoystickButton(joystick, XBoxButton.Y.getValue());
        JoystickButton driverLB = new JoystickButton(joystick, XBoxButton.LB.getValue());
        JoystickButton driverRB = new JoystickButton(joystick, XBoxButton.RB.getValue());
        JoystickButton driverBack = new JoystickButton(joystick, XBoxButton.Back.getValue());
        JoystickButton driverStart = new JoystickButton(joystick, XBoxButton.Start.getValue());
        JoystickButton driverLeftStick = new JoystickButton(joystick, XBoxButton.LeftStick.getValue());
        JoystickButton driverRightStick = new JoystickButton(joystick, XBoxButton.RightStick.getValue());
        
        driverX.whileHeld(new DriveForward());
        driverA.whileHeld(new DriveReverse());
		
		return joystick;
	}
	
    public double getDriverAxis(XBoxAxis axis) {
        return driverJoystick.getRawAxis(axis.getValue());
    }
}