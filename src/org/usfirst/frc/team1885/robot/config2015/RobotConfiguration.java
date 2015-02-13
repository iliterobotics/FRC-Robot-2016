package org.usfirst.frc.team1885.robot.config2015;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotConfiguration {

    public static void configureRobot(){
        DriverInputControl.getInstance().addJoystick( RobotJoystickType.LEFT_DRIVE, 0 );
        DriverInputControl.getInstance().addJoystick( RobotJoystickType.RIGHT_DRIVE, 1 );
        DriverInputControl.getInstance().addJoystick( RobotJoystickType.CONTROLLER, 2 );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, Joystick.AxisType.kTwist ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.WRIST_ROTATION, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, Joystick.AxisType.kX ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.WRIST_EXTENSION, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, Joystick.AxisType.kY ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.CLAW, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 4) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 2 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_DOWN, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 3 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_PICKUP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 8 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_RELEASE, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 7 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_INCREMENT, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 6 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_DECREMENT, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 5 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_NUDGE_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 8 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_NUDGE_DOWN, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 7 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_RESET, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 2 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.GEAR_SHIFT, new JoystickButtonMatch( RobotJoystickType.RIGHT_DRIVE, 2 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RIGHT_DRIFT, new JoystickButtonMatch( RobotJoystickType.RIGHT_DRIVE, 1 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.LEFT_DRIFT, new JoystickButtonMatch( RobotJoystickType.LEFT_DRIVE, 1 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.HARD_STOP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 1 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.NUDGE, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, true) );
        SensorInputControl.getInstance().addSensor( SensorType.DRIVE_TRAIN_LEFT_ENCODER, 0, 1 );
        SensorInputControl.getInstance().addSensor( SensorType.DRIVE_TRAIN_RIGHT_ENCODER, 2, 3 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_ENCODER, 4, 5 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_ENCODER, 6, 7 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_UPPER_LIMIT_SWITCH, 8 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_LOWER_LIMIT_SWITCH, 9 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_UPPER_LIMIT, 10 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_LOWER_LIMIT, 11 );
        SensorInputControl.getInstance().addSensor( SensorType.LINE_SENSOR, 12 );
        SensorInputControl.getInstance().addSensor( SensorType.MAGNET_SENSOR, 13 );
        SensorInputControl.getInstance().addSensor( SensorType.TOUCH_SENSOR1, 14 );
        SensorInputControl.getInstance().addSensor( SensorType.TOUCH_SENSOR2, 15 );
        SensorInputControl.getInstance().setUpNAVX( (byte)(50), SerialPort.Port.kMXP );
        SensorInputControl.getInstance().addLidarSensor( I2C.Port.kMXP );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.LEFT_DRIVE, 0 );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.LEFT_DRIVE, 2 );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.RIGHT_DRIVE, 1 );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.RIGHT_DRIVE, 3 );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.TOTE_LIFT, 4 );
        RobotControl.getInstance().addTalonOutput( RobotMotorType.RECYCLE_LIFT, 7 );
        RobotControl.getInstance().addPneumaticOutput(RobotPneumaticType.GEAR_SHIFTER_PNEUMATIC, 2);
        RobotControl.getInstance().addPneumaticOutput(RobotPneumaticType.TOTE_LIFT_STOP, 3);
        RobotControl.getInstance().addPneumaticOutput(RobotPneumaticType.WRIST_ROTATION, 1, 7);
        RobotControl.getInstance().addPneumaticOutput(RobotPneumaticType.WRIST_EXTENSION, 0, 6);
        RobotControl.getInstance().addPneumaticOutput(RobotPneumaticType.GRABBER_PNEUMATIC, 5);
        
    }
}
