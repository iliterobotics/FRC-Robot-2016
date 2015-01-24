package org.usfirst.frc.team1885.robot.common.type;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.input.DriverInputControl;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickButtonMap {
	private HashMap<RobotButtonType, Integer> controllerButtonMap;
	private static JoystickButtonMap instance;
	
	protected JoystickButtonMap()
	{
		controllerButtonMap = new HashMap<RobotButtonType, Integer>();
	}
	
	public static JoystickButtonMap getInstance()
	{
		if(instance == null)
		{
			instance = new JoystickButtonMap();
		}
		return instance;
	}
	
	public void addControllerButton(RobotButtonType buttonType, Integer rawButton) {
		controllerButtonMap.put(buttonType, rawButton);
	}
	
	public HashMap getControllerButtonMap() {
		return controllerButtonMap;
	}
}
