package org.usfirst.frc.team1885.robot.config2015;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RobotConfiguration {

    public RobotConfiguration(){
        DriverInputControl.getInstance();
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 0 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_DOWN, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 1 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 2 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_DOWN, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 3 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_PICKUP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 4 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_RELEASE, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 5 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_PICKUP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 6 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.RECYCLE_BIN_LIFT_RELEASE, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 7 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_INCREMENT, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 8 ) );
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.GEAR_SHIFT, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 9 ) );
        SensorInputControl.getInstance().addSensor( SensorType.DRIVE_TRAIN_LEFT_ENCODER, 1, 2 );
        SensorInputControl.getInstance().addSensor( SensorType.DRIVE_TRAIN_RIGHT_ENCODER, 3, 4 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_ENCODER, 5, 6 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_ENCODER, 7, 8 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_UPPER_LIMIT_SWITCH, 10 );
        SensorInputControl.getInstance().addSensor( SensorType.TOTE_LOWER_LIMIT_SWITCH, 11 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_UPPER_LIMIT, 12 );
        SensorInputControl.getInstance().addSensor( SensorType.RECYCLE_BIN_LOWER_LIMIT, 13 );
    }
}
