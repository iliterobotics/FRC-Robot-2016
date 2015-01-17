package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;

import edu.wpi.first.wpilibj.Joystick;

public class DriverInputControl
{
	private static DriverInputControl instance = null;
	
	private Map<RobotJoystickType, Joystick> joystickMap;
	private Map<RobotButtonType, Integer> buttonMap;

	
	public static final double DEADZONE = 0.1;
	protected DriverInputControl()
	{
		
        joystickMap = new HashMap<RobotJoystickType, Joystick>();
        buttonMap = new HashMap<RobotButtonType, Integer>();

	}
	public static DriverInputControl getInstance()
	{
		if(instance == null)
		{
			instance = new DriverInputControl();
		}
		return instance;
	}
	
	public void addJoystick(RobotJoystickType type, int port)
	{
		joystickMap.put(type, new Joystick(port));
	}
	
    public static double deadzone(double axis)
    {
    	if(Math.abs(axis) < DEADZONE)
    	{
    		return 0;
    	}
    	return axis;
    }    	
    public double getLeftDrive()
    {
    	double axis = joystickMap.get(RobotJoystickType.LEFT_DRIVE).getAxis(Joystick.AxisType.kY);
    	return deadzone(axis);
    }
    public double getRightDrive()
    {
    	double axis = joystickMap.get(RobotJoystickType.RIGHT_DRIVE).getAxis(Joystick.AxisType.kY);
    	return deadzone(axis);
    }
    public Joystick getJoystick(RobotJoystickType joystickType)
    {
    	return joystickMap.get(joystickType);
    }
    public boolean getControllerButton(int buttonLocation)
    {
    	return joystickMap.get(RobotJoystickType.CONTROLLER).getRawButton(buttonLocation);
    }

}