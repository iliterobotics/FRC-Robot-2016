package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotConfiguration {
	public static void configureRobot()
	{
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.CONTROLLER, new Joystick(2) );
		
		SensorInputControlSRX.getInstance().createNavX( SerialPort.Port.kOnboard );
		
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 3);
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 4);
		
		JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_JOINT_A_CLOCK, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 1 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_JOINT_A_COUNTER, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 2 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_JOINT_B_CLOCK, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 4 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_JOINT_B_COUNTER, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 3 ) );

		RobotControlWithSRX.getInstance().addTalonSensor(SensorType.JOINT_B_POTENTIOMETER, 2);
        
        //TODO change to match actual input
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_A, 1);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_B, 2);
	}
}