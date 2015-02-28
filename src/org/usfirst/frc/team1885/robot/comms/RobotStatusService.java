package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.modules.lift.RecycleBinLift;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;

public class RobotStatusService 
{
	private ToteLift tl = ToteLift.getInstance();
	private RecycleBinLift rb = RecycleBinLift.getInstance();
	private DrivetrainControl dt = DrivetrainControl.getInstance();
	private SensorInputControl sic = SensorInputControl.getInstance();	

	private TelemetryMessage tm = new TelemetryMessage();
	
	public void update() {
		setToteLiftSpeed();
		setToteReset();
		setToteState();
		setToteBrake();
		setRecycleBinHasBin();
		setRecycleBinSpeed();
		setRecycleBinState();
		setLeftDriveSpeed();
		setRightDriveSpeed();
		setDriveMode();
		setIsTurning();
		setUpperRecycleLimitSensor();
		setLowerRecycleLimitSensor();
		setUpperToteLimitSwitch();
		setLowerToteLimitSwitch();
		setLeftTouchSensor();
		setRightTouchSensor();
		setLineSensor();
		setMagnetSensor();
	}
	
	public void setToteLiftSpeed()
	{
		tm.setToteLiftSpeed(tl.getSpeed());
	}
	public void setToteReset()
	{
		tm.setToteResetting(tl.getIsResetting());
	}
	public void setToteState()
	{
		tm.setToteState(tl.getMotorState());
	}
	public void setToteBrake()
	{
		tm.setToteBrake(tl.getIsBraked());
	}
	
	public void setRecycleBinHasBin()
	{
		tm.setHasBin(rb.hasBin());
	}
	public void setRecycleBinSpeed()
	{
		tm.setRecycleBinLiftSpeed(rb.getSpeed());
	}
	public void setRecycleBinState()
	{
		tm.setRecycleBinState(rb.getMotorState());
	}
	
	public void setLeftDriveSpeed()
	{
		tm.setLeftDriveSpeed(dt.getLeftDriveSpeed());
	}
	public void setRightDriveSpeed()
	{
		tm.setRightDriveSpeed(dt.getRightDriveSpeed());
	}
	public void setDriveMode()
	{
		tm.setGearState(dt.getGearState());
	}
	public void setIsTurning()
	{
		tm.setTurning(dt.getIsTurning());
	}
	
	public void setUpperRecycleLimitSensor()
	{
		tm.setUpperRecycleLimitSwitch(sic.getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get());
	}
	public void setLowerRecycleLimitSensor()
	{
		tm.setLowerRecycleLimitSwitch(sic.getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get());
	}
	public void setUpperToteLimitSwitch()
	{
		tm.setUpperToteLimitSwitch(sic.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get());
	}
	public void setLowerToteLimitSwitch()
	{
		tm.setLowerToteLimitSwitch(false);
	}
	public void setLeftTouchSensor()
	{
		tm.setLeftTouchSensor(sic.getLimitSwitch(SensorType.TOUCH_SENSOR_TOTE_LEFT).get());
	}
	public void setRightTouchSensor()
	{
		tm.setRightTouchSensor(sic.getLimitSwitch(SensorType.TOUCH_SENSOR_TOTE_RIGHT).get());
	}
	public void setLineSensor()
	{
		tm.setLineSensor(sic.getLimitSwitch(SensorType.LINE_SENSOR).get());
	}
	public void setMagnetSensor()
	{
		tm.setMagnetSensor(sic.getLimitSwitch(SensorType.MAGNET_SENSOR).get());
	}

	public TelemetryMessage getTm() {
		return tm;
	}

	public void setTm(TelemetryMessage tm) {
		this.tm = tm;
	}
	
	
}
