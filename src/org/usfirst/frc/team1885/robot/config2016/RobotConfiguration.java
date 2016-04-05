
package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;

public class RobotConfiguration {
	public static void configureRobot()
	{
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.CONTROLLER, new Joystick(2) );
		
		JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_MOVE_Y, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, Joystick.AxisType.kThrottle ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.ARM_MOVE_X, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, Joystick.AxisType.kTwist) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RESET_BUTTON, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 9));
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.DRAWBRIDGE_BREACH, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 1));
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.SALLYPORT_BREACH, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 2));
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.CHEVAL_BREACH, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 3));
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.INCREMENT_BREACH_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 6));
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.INCREMENT_BREACH_DOWN, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 8));
        
        
        
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_LEFT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 1));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_DOWN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 2));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_RIGHT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 3));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_UP, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 4));
        
        
        
        //TODO change to match actual input
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_A, 6);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_B, 5);
        
        RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.ARM_JOINT_A, SensorType.JOINT_A_CTRE_ABSOLUTE, 1);
		RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.ARM_JOINT_B, SensorType.JOINT_B_CTRE_ABSOLUTE, 2);
	}
}
