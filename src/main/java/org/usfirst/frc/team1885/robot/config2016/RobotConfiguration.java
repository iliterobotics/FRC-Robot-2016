package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class RobotConfiguration {
	public static void configureRobot()
	{
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 3);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 4);
	}
}
