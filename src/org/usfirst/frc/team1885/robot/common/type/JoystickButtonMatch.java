package org.usfirst.frc.team1885.robot.common.type;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickButtonMatch {
    private RobotJoystickType joystickType;
    private int port;
    private Joystick.AxisType axisType;

    public JoystickButtonMatch(RobotJoystickType joystickType, int port) {
        this.joystickType = joystickType;
        this.port = port;
    }

    public JoystickButtonMatch(RobotJoystickType joystickType,
            Joystick.AxisType axisType) {
        this.joystickType = joystickType;
        this.axisType = axisType;
    }

    public RobotJoystickType getJoystickType() {
        return joystickType;
    }

    public int getPort() {
        return port;
    }

    public Joystick.AxisType getAxisType() {
        return axisType;
    }
}
