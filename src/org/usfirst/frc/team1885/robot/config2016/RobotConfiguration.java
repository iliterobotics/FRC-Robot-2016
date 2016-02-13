
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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotConfiguration {

    public static final double WHEEL_DIAMETER = 8.0;
    public static final int ARM_JOINT_A_PORT = 1; // 5 on real robot
    public static final int ARM_JOINT_B_PORT = 2; // 6 on real robot
    public static final double MAX_SPEED = .8;
    public static void configureRobot() {
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.LEFT_DRIVE, new Joystick(0));
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.RIGHT_DRIVE, new Joystick(1));
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.CONTROLLER, new Joystick(2));

        SensorInputControlSRX.getInstance().createNavX(SerialPort.Port.kMXP);

        // Temp comment because we don't have the actual robot. We just have the
        // test board...
        // RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE,
        // 1);
        // RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE,
        // 2);
        // RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.LEFT_DRIVE,
        // 3);
        // RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.RIGHT_DRIVE,
        // 4);
        // TODO change to match actual input

        // RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.LEFT_DRIVE,
        // SensorType.LEFT_ENCODER, 1);
        // RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.RIGHT_DRIVE,
        // SensorType.RIGHT_ENCODER, 2);
        RobotControlWithSRX.getInstance().addTalonSensor(
                RobotMotorType.ARM_JOINT_A, SensorType.JOINT_A_POTENTIOMETER,
                ARM_JOINT_A_PORT);
        RobotControlWithSRX.getInstance().addTalonSensor(
                RobotMotorType.ARM_JOINT_B, SensorType.JOINT_B_POTENTIOMETER,
                ARM_JOINT_B_PORT);

        JoystickButtonMap.getInstance().addControllerButton(
                RobotButtonType.ARM_MOVE_Y,
                new JoystickButtonMatch(RobotJoystickType.CONTROLLER,
                        Joystick.AxisType.kTwist));
        JoystickButtonMap.getInstance().addControllerButton(
                RobotButtonType.ARM_MOVE_X,
                new JoystickButtonMatch(RobotJoystickType.CONTROLLER,
                        Joystick.AxisType.kThrottle));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.RESET_BUTTON, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 9));
        
        
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_LEFT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 1));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_DOWN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 2));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_RIGHT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 3));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_UP, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 4));

    }
}
