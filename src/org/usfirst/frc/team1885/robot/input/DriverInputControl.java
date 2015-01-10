package org.usfirst.frc.team1885.robot.input;

import edu.wpi.first.wpilibj.Joystick;

public class DriverInputControl
{
	private Joystick leftStick;  
	private Joystick rightStick; 

	
	public static final double DEADZONE = 0.1;
	public DriverInputControl(int leftJoystick, int rightJoystick)
	{
		
        leftStick = new Joystick(leftJoystick);
        rightStick = new Joystick(rightJoystick);

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
    	double axis = leftStick.getAxis(Joystick.AxisType.kY);
    	return deadzone(axis);
    }
    public double getRightDrive()
    {
    	double axis = rightStick.getAxis(Joystick.AxisType.kY);
    	return deadzone(axis);
    }

}
