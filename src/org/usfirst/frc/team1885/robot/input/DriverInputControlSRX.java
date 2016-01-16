package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.output.RobotControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class DriverInputControlSRX {
	private HashMap<RobotJoystickType, Joystick> joystickMap;
	private static DriverInputControlSRX instance = null;
	private static final double DEADZONE = .1;
	private double leftDriveSpeed,rightDriveSpeed;
	protected DriverInputControlSRX() {
        joystickMap = new HashMap<RobotJoystickType, Joystick>();
        leftDriveSpeed = 0;
        rightDriveSpeed = 0;
    }
	public static DriverInputControlSRX getInstance(){
	    if(instance == null){
	        instance = new DriverInputControlSRX();
	    }
	    return instance;
	}
	public static double deadzone(double axis) {
        if (Math.abs(axis) < DEADZONE) {
            return 0;
        }
        return axis;
    }
	public double getLeftDrive() {
        return this.leftDriveSpeed;
    }
    public double getRightDrive() {
        return this.rightDriveSpeed;
    }
    public Joystick getJoystick(RobotJoystickType joystickType) {
        return joystickMap.get(joystickType);
    }
    public boolean addJoystick(RobotJoystickType joystickType, Joystick joystick)
    {
    	joystickMap.put(joystickType, joystick);
    	return true;
    }
    public void update()
    {
        this.update(this.getJoystick(RobotJoystickType.LEFT_DRIVE).getAxis(AxisType.kY), this.getJoystick(RobotJoystickType.RIGHT_DRIVE).getAxis(AxisType.kY));
    }
    public void update(double leftJoystick, double rightJoystick)
    {    
        this.rightDriveSpeed = deadzone(rightJoystick);
        this.leftDriveSpeed = deadzone(leftJoystick);
        RobotControlWithSRX.getInstance().updateDriveSpeed(rightDriveSpeed, leftDriveSpeed);
    }
}
