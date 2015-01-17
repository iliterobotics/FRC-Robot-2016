package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;

import edu.wpi.first.wpilibj.Joystick;

public class DriverInputControl
{
	
	private Map<RobotJoystickType, Joystick> joystickMap;

	
	public static final double DEADZONE = 0.1;
	public DriverInputControl()
	{
		
        joystickMap = new HashMap<RobotJoystickType, Joystick>();

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

}