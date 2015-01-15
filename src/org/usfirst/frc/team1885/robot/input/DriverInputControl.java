package org.usfirst.frc.team1885.robot.input;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class DriverInputControl
{
	private Joystick leftStick;  
	private Joystick rightStick; 

	private Encoder	drive_train_left_encoder;
	private Encoder drive_train_right_encoder;
	private Encoder recycle_bin_lift_left_encoder;
	private Encoder	recycle_bin_lift_right_encoder;
	private Encoder	tote_lift_left_encoder;
	private Encoder	tote_lift_right_encoder;
	
	private DigitalInput tote_lift_top_limit_switch;
	private DigitalInput tote_lift_bottom_limit_switch;
	
	public static final double DEADZONE = 0.1;
	public DriverInputControl(int leftJoystick, int rightJoystick, int drive_train_left_encoder_port_one, 
			int drive_train_left_encoder_port_two, int drive_train_right_encoder_port_one, 
			int drive_train_right_encoder_port_two, int recycle_bin_lift_left_encoder_port_one, 
			int recycle_bin_lift_left_encoder_port_two, int recycle_bin_lift_right_encoder_port_one, 
			int recycle_bin_lift_right_encoder_port_two, int tote_lift_left_encoder_port_one, 
			int tote_lift_left_encoder_port_two, int tote_lift_right_encoder_port_one, 
			int tote_lift_right_encoder_port_two, int tote_lift_top_limit_switch_port, 
			int tote_lift_bottom_limit_switch_port)
	{
		
        leftStick = new Joystick(leftJoystick);
        rightStick = new Joystick(rightJoystick);
        drive_train_left_encoder = new Encoder(drive_train_left_encoder_port_one, drive_train_left_encoder_port_two);
        drive_train_right_encoder = new Encoder(drive_train_right_encoder_port_one, drive_train_right_encoder_port_two);
        recycle_bin_lift_left_encoder = new Encoder(recycle_bin_lift_left_encoder_port_one, recycle_bin_lift_left_encoder_port_two);
        recycle_bin_lift_right_encoder = new Encoder(recycle_bin_lift_right_encoder_port_one, recycle_bin_lift_right_encoder_port_two);
        tote_lift_left_encoder = new Encoder(tote_lift_left_encoder_port_one, tote_lift_left_encoder_port_two);
        tote_lift_right_encoder = new Encoder(tote_lift_right_encoder_port_one, tote_lift_right_encoder_port_two);
        tote_lift_top_limit_switch = new DigitalInput(tote_lift_top_limit_switch_port);
        tote_lift_bottom_limit_switch = new DigitalInput(tote_lift_bottom_limit_switch_port);
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
    public int getDriveTrainLeftEncoder()
    {
    	return drive_train_left_encoder.get();
    }
    public int getDriveTrainRightEncoder()
    {
    	return drive_train_right_encoder.get();
    }
    public int getRecycleBinLiftLeftEncoder()
    {
    	return recycle_bin_lift_left_encoder.get();
    }
    public int getRecycleBinLiftRightEncoder()
    {
    	return recycle_bin_lift_right_encoder.get();
    }
    public int getToteLiftLeftEncoder()
    {
    	return tote_lift_left_encoder.get();
    }
    public int getToteLiftRightEncoder()
    {
    	return tote_lift_right_encoder.get();
    }
}
