
package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * 
 * @author ILITE Robotics
 * @version This class keeps track of all joy stick mappings and sensor-to-talon
 *          mappings for the robot Also keeps track of ports for motors
 */
public class RobotConfiguration {
    public static final double WHEEL_DIAMETER = 9.0;
    public static final int ARM_JOINT_A_PORT = 1; // 5 on real robot
    public static final int ARM_JOINT_B_PORT = 2; // 6 on real robot
    public static final double MAX_SPEED = .8;

    /**
     * Method adds all joy stick mappings and sensor-to-talon mappings
     */
    public static void configureRobot() {
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.LEFT_DRIVE, new Joystick(0));
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.RIGHT_DRIVE, new Joystick(1));
        DriverInputControlSRX.getInstance()
                .addJoystick(RobotJoystickType.CONTROLLER, new Joystick(2));
     
        JoystickButtonMap.getInstance().addControllerButton(
                RobotButtonType.GEAR_SHIFT, 
                new JoystickButtonMatch(RobotJoystickType.LEFT_DRIVE, 3));
//        
        SensorInputControlSRX.getInstance().createNavX(SerialPort.Port.kMXP);
        SensorInputControlSRX.getInstance().addPressureSensor(0);
        
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.FLYWHEEL_IN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 5));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.FLYWHEEL_OUT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 7));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TILT_UP, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 4));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TILT_DOWN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 2));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TWIST_LEFT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 1));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_TWIST_RIGHT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 3));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.SHOOTER_RESET, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 6));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.READY_HIGH, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,1));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.READY_LOW, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,2));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.AIM, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,3));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.FIRE, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,8));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INTAKE_SOLENOID, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,1));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INTAKE_IN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,3));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INTAKE_OUT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER,5));
        
        
        
        SensorInputControlSRX.getInstance().addEncoder(RobotMotorType.FLYWHEEL_LEFT,SensorType.FLYWHEEL_LEFT_ENCODER, 8);
        SensorInputControlSRX.getInstance().addEncoder(RobotMotorType.FLYWHEEL_RIGHT,SensorType.FLYWHEEL_RIGHT_ENCODER, 9);
        SensorInputControlSRX.getInstance().addPotentiometer(RobotMotorType.SHOOTER_TILT,SensorType.SHOOTER_TILT_POTENTIOMETER, 10);
        SensorInputControlSRX.getInstance().addEncoder(RobotMotorType.SHOOTER_TWIST,SensorType.SHOOTER_TWIST_ENCODER, 11);

        
        
        //Talon Outputs for Robot
        RobotControlWithSRX.getInstance()
                .addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
        RobotControlWithSRX.getInstance()
                .addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
        RobotControlWithSRX.getInstance()
                .addTalonOutput(RobotMotorType.LEFT_DRIVE, 3);
        RobotControlWithSRX.getInstance()
                .addTalonOutput(RobotMotorType.RIGHT_DRIVE, 4);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_A, 5);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.ARM_JOINT_A, 6);
        RobotControlWithSRX.getInstance()
                .addTalonOutput(RobotMotorType.ACTIVE_INTAKE, 7);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.FLYWHEEL_LEFT, 8);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.FLYWHEEL_RIGHT, 9);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.SHOOTER_TILT, 10);
        RobotControlWithSRX.getInstance().addTalonOutput(RobotMotorType.SHOOTER_TWIST, 11);
        
        
        //Solenoids and DoubleSolenoids
        RobotControlWithSRX.getInstance()
                .addDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER, 0);
        RobotControlWithSRX.getInstance().addSingleSolenoid(RobotPneumaticType.GEAR_SHIFT, 3);
        RobotControlWithSRX.getInstance().addSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER, 4);
        
        
        //Talon Sensors
        RobotControlWithSRX.getInstance().addTalonSensor(
                RobotMotorType.LEFT_DRIVE, SensorType.LEFT_ENCODER, 1);
        RobotControlWithSRX.getInstance().addTalonSensor(
                RobotMotorType.RIGHT_DRIVE, SensorType.RIGHT_ENCODER, 2);

        
        
        
        // TODO change to match actual input
        
        
//        RobotControlWithSRX.getInstance().addTalonSensor(
//                RobotMotorType.ARM_JOINT_A, SensorType.JOINT_A_POTENTIOMETER,
//                ARM_JOINT_A_PORT);
//        RobotControlWithSRX.getInstance().addTalonSensor(
//                RobotMotorType.ARM_JOINT_B, SensorType.JOINT_B_POTENTIOMETER,
//                ARM_JOINT_B_PORT);

        JoystickButtonMap.getInstance().addControllerButton(
                RobotButtonType.ARM_MOVE_Y,
                new JoystickButtonMatch(RobotJoystickType.CONTROLLER,
                        Joystick.AxisType.kTwist));
        JoystickButtonMap.getInstance().addControllerButton(
                RobotButtonType.ARM_MOVE_X,
                new JoystickButtonMatch(RobotJoystickType.CONTROLLER,
                        Joystick.AxisType.kThrottle));
        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.RESET_BUTTON, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 9));
        
        
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_LEFT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 8));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_DOWN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 7));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_RIGHT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 6));
//        JoystickButtonMap.getInstance().addControllerButton(RobotButtonType.INCREMENT_ARM_UP, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 5));
    }
}
