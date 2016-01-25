package org.usfirst.frc.team1885.robot.common.type;

import java.util.HashMap;

public class JoystickButtonMap {
    private HashMap<RobotButtonType, JoystickButtonMatch> buttonMap;

    private static JoystickButtonMap instance;

    protected JoystickButtonMap() {
        buttonMap = new HashMap<RobotButtonType, JoystickButtonMatch>();
    }

    public static JoystickButtonMap getInstance() {
        if (instance == null) {
            instance = new JoystickButtonMap();
        }
        return instance;
    }

    public void addControllerButton(RobotButtonType buttonType,
            JoystickButtonMatch joystickButton) {
        buttonMap.put(buttonType, joystickButton);
    }

    public HashMap<RobotButtonType, JoystickButtonMatch> getButtonMap() {
        return buttonMap;
    }
}
