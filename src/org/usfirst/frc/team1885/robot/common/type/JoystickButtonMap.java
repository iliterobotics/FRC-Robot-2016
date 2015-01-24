package org.usfirst.frc.team1885.robot.common.type;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.input.DriverInputControl;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickButtonMap {
	private HashMap<RobotButtonType, JoystickButtonMatch> buttonMap;
	
	
	private static JoystickButtonMap instance;
	
	protected JoystickButtonMap()
	{
		buttonMap = new HashMap<RobotButtonType, JoystickButtonMatch>();
	}
	
	public static JoystickButtonMap getInstance()
	{
		if(instance == null)
		{
			instance = new JoystickButtonMap();
		}
		return instance;
	}
	
	public void addControllerButton(RobotButtonType buttonType, JoystickButtonMatch joystickButton) {
		buttonMap.put(buttonType, joystickButton);
	}
	
	public HashMap getButtonMap() {
		return buttonMap;
	}
}
