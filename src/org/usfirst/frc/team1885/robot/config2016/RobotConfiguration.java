package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;

public class RobotConfiguration {
	public static void configureRobot()
	{
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.CONTROLLER, new Joystick(2) );
		
//		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
//        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
//        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 3);
//        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 4);
        
        
		JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.FLYWHEEL_IN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 5));
		JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.FLYWHEEL_OUT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 7));
		JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TILT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, Joystick.AxisType.kY));
		JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TWIST, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, Joystick.AxisType.kX));
		
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.FLYWHEEL_LEFT, 5);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.FLYWHEEL_RIGHT, 6);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.SHOOTER_TILT, 7);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.SHOOTER_TWIST, 8);
		
//		SensorInputControlSRX.getInstance().addLidarSensor( I2C.Port.kMXP );
//        SensorInputControlSRX.getInstance().getLidarSensor(SensorType.LIDAR).start(1000);
	}
}
