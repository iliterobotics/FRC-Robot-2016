package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.manipulator.ClawControl;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.modules.lift.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.lift.RecycleBinLift;
import org.usfirst.frc.team1885.robot.modules.lift.ToteLift;

public class RobotStatusService 
{
	private ToteLift tl = ToteLift.getInstance();
	private RecycleBinLift rb = RecycleBinLift.getInstance();
	private DrivetrainControl dt = DrivetrainControl.getInstance();
	private SensorInputControl sic = SensorInputControl.getInstance();
	private ClawControl ccInstance = ClawControl.getInstance();
	private ActiveIntake aiInstance = ActiveIntake.getInstance();

	private RobotInfoMessage robotInfoMessage = new RobotInfoMessage();
	
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
		updateClaw();
		updateActiveIntake();
	}
	
	public void updateActiveIntake(){
	    this.robotInfoMessage.setActiveMotorState(aiInstance.getMotorState());
	}
	
	public void updateClaw() {
		this.robotInfoMessage.setRotationState(ccInstance.clawWristRotationState());
		this.robotInfoMessage.setExtensionState(ccInstance.clawWristExtentionState());
		this.robotInfoMessage.setPinchState(ccInstance.clawPinchState());
	}
	
	public void setToteLiftSpeed()
	{
		robotInfoMessage.setToteLiftSpeed(tl.getSpeed());
	}
	public void setToteReset()
	{
		robotInfoMessage.setToteResetting(tl.getIsResetting());
	}
	public void setToteState()
	{
		robotInfoMessage.setToteState(tl.getMotorState());
	}
	public void setToteBrake()
	{
		robotInfoMessage.setToteBrake(tl.getIsBraked());
	}
	
	public void setRecycleBinHasBin()
	{
		robotInfoMessage.setHasBin(rb.hasBin());
	}
	public void setRecycleBinSpeed()
	{
		robotInfoMessage.setRecycleBinLiftSpeed(rb.getSpeed());
	}
	public void setRecycleBinState()
	{
		robotInfoMessage.setRecycleBinState(rb.getMotorState());
	}
	
	public void setLeftDriveSpeed()
	{
		robotInfoMessage.setLeftDriveSpeed(dt.getLeftDriveSpeed());
	}
	public void setRightDriveSpeed()
	{
		robotInfoMessage.setRightDriveSpeed(dt.getRightDriveSpeed());
	}
	public void setDriveMode()
	{
		robotInfoMessage.setGearState(dt.getGearState());
	}
	public void setIsTurning()
	{
		robotInfoMessage.setTurning(dt.getIsTurning());
	}
	
	public void setUpperRecycleLimitSensor()
	{
		robotInfoMessage.setUpperRecycleLimitSwitch(sic.getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get());
	}
	public void setLowerRecycleLimitSensor()
	{
		robotInfoMessage.setLowerRecycleLimitSwitch(sic.getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get());
	}
	public void setUpperToteLimitSwitch()
	{
		robotInfoMessage.setUpperToteLimitSwitch(sic.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get());
	}
	public void setLowerToteLimitSwitch()
	{
		robotInfoMessage.setLowerToteLimitSwitch(false);
	}
	public void setLeftTouchSensor()
	{
		robotInfoMessage.setLeftTouchSensor(sic.getLimitSwitch(SensorType.TOUCH_SENSOR_TOTE_LEFT).get());
	}
	public void setRightTouchSensor()
	{
		robotInfoMessage.setRightTouchSensor(sic.getLimitSwitch(SensorType.TOUCH_SENSOR_TOTE_RIGHT).get());
	}
	public void setLineSensor()
	{
		robotInfoMessage.setLineSensor(sic.getLimitSwitch(SensorType.LINE_SENSOR).get());
	}
	public void setMagnetSensor()
	{
		robotInfoMessage.setMagnetSensor(sic.getLimitSwitch(SensorType.MAGNET_SENSOR).get());
	}

	public RobotInfoMessage getTm() {
		return robotInfoMessage;
	}

	public void setTm(RobotInfoMessage rIM) {
		this.robotInfoMessage = rIM;
	}
	
	
}
