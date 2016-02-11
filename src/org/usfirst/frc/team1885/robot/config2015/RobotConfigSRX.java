package org.usfirst.frc.team1885.robot.config2015;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import com.sun.xml.internal.ws.api.addressing.AddressingVersion.EPR;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotConfigSRX {
	public static void configureRobot()
	{
	    try {
//		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, new Joystick(0) );
//		DriverInputControlSRX.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, new Joystick(1) );
		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);

		RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		//RobotControlWithSRX.getInstance().addTalonSensor(SensorType.DRIVE_TRAIN_ENCODER, 1);
		//RobotControlWithSRX.getInstance().addTalonSensor(SensorType.ULTRASONIC, 1);
		RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.TEST_MOTOR, SensorType.DRIVE_TRAIN_ENCODER, 4);
		SensorInputControlSRX.getInstance().addLidarSensor( I2C.Port.kOnboard );
        SensorInputControlSRX.getInstance().getLidarSensor().start();
        SensorInputControlSRX.getInstance().createNavX(SerialPort.Port.kMXP);
	    } catch(Throwable e) {
	        System.err.println("OMG!!");
	        e.printStackTrace();
	    }
	}
}
