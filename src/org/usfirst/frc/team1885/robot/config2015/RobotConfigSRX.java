package org.usfirst.frc.team1885.robot.config2015;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

public class RobotConfigSRX {
	public static void configureRobot()
	{
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		//RobotControlWithSRX.getInstance().addTalonSensor(SensorType.DRIVE_TRAIN_ENCODER, 1);
		RobotControlWithSRX.getInstance().addTalonSensor(SensorType.LIMIT_SWITCH, 1);
		SensorInputControlSRX.getInstance().addLidarSensor( I2C.Port.kMXP );
        SensorInputControlSRX.getInstance().getLidarSensor().start();
	}
}