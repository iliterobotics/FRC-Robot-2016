package org.usfirst.frc.team1885.robot.config2015;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;

public class RobotConfiguration {

    public RobotConfiguration(){
        DriverInputControl.getInstance();
        JoystickButtonMap.getInstance().addControllerButton( RobotButtonType.TOTE_LIFT_UP, new JoystickButtonMatch( RobotJoystickType.CONTROLLER, 3 ) );
        SensorInputControl.getInstance();
    }
}
