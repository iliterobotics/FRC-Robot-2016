package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class RobotInfoMessage extends Message {
	private static final long serialVersionUID = 751558634718818846L;
	// ToteLift
	private double toteLiftSpeed;
	private boolean toteResetting;
	private MotorState toteState;
	private boolean toteBrake;
	// RecycleLift
	private boolean hasBin;
	private double recycleBinLiftSpeed;
	private MotorState recycleBinState;
	// DriveTrain
	private double leftDriveSpeed;
	private double rightDriveSpeed;
	private boolean driveMode;
	private GearState gearState;
	private boolean isTurning;
	// Sensors
	private boolean upperRecycleLimitSwitch;
	private boolean lowerRecycleLimitSwitch;
	private boolean upperToteLimitSwitch;
	private boolean lowerToteLimitSwitch;
	private boolean leftTouchSensor;
	private boolean rightTouchSensor;
	private boolean lineSensor;
	private boolean magnetSensor;
	// Claw
	private boolean rotationState;
	private boolean extensionState;
	private boolean pinchState;
	// Active Intake
	private double activeMotorIntakeLeft;
	private double activeMotorIntakeRight;
	private MotorState activeMotorStateLeft;
	private MotorState activeMotorStateRight;
	
	
	public RobotInfoMessage() {
		super(ServerMessageType.ROBOTINFO_MSG);
	}
	
	public boolean isRotationState() {
		return rotationState;
	}

	public void setRotationState(boolean rotationState) {
		this.rotationState = rotationState;
	}

	public boolean isExtensionState() {
		return extensionState;
	}

	public void setExtensionState(boolean extensionState) {
		this.extensionState = extensionState;
	}

	public boolean isPinchState() {
		return pinchState;
	}

	public void setPinchState(boolean pinchState) {
		this.pinchState = pinchState;
	}

	public double getToteLiftSpeed() {
		return toteLiftSpeed;
	}

	public void setToteLiftSpeed(double toteLiftSpeed) {
		this.toteLiftSpeed = toteLiftSpeed;
	}

	public boolean isToteResetting() {
		return toteResetting;
	}

	public void setToteResetting(boolean toteResetting) {
		this.toteResetting = toteResetting;
	}

	public MotorState getToteState() {
		return toteState;
	}

	public void setToteState(MotorState toteState) {
		this.toteState = toteState;
	}

	public boolean isToteBrake() {
		return toteBrake;
	}

	public void setToteBrake(boolean toteBrake) {
		this.toteBrake = toteBrake;
	}

	public boolean isHasBin() {
		return hasBin;
	}

	public void setHasBin(boolean hasBin) {
		this.hasBin = hasBin;
	}

	public double getRecycleBinLiftSpeed() {
		return recycleBinLiftSpeed;
	}

	public void setRecycleBinLiftSpeed(double recycleBinLiftSpeed) {
		this.recycleBinLiftSpeed = recycleBinLiftSpeed;
	}

	public MotorState getRecycleBinState() {
		return recycleBinState;
	}

	public void setRecycleBinState(MotorState state) {
		this.recycleBinState = state;
	}

	public double getLeftDriveSpeed() {
		return leftDriveSpeed;
	}

	public void setLeftDriveSpeed(double leftDriveSpeed) {
		this.leftDriveSpeed = leftDriveSpeed;
	}

	public double getRightDriveSpeed() {
		return rightDriveSpeed;
	}

	public void setRightDriveSpeed(double rightDriveSpeed) {
		this.rightDriveSpeed = rightDriveSpeed;
	}

	public boolean isDriveMode() {
		return driveMode;
	}

	public void setDriveMode(boolean driveMode) {
		this.driveMode = driveMode;
	}

	public GearState getGearState() {
		return gearState;
	}

	public void setGearState(GearState gearState) {
		this.gearState = gearState;
	}

	public boolean isTurning() {
		return isTurning;
	}

	public void setTurning(boolean isTurning) {
		this.isTurning = isTurning;
	}

	public boolean isUpperRecycleLimitSwitch() {
		return upperRecycleLimitSwitch;
	}

	public void setUpperRecycleLimitSwitch(boolean upperRecycleLimitSwitch) {
		this.upperRecycleLimitSwitch = upperRecycleLimitSwitch;
	}

	public boolean isLowerRecycleLimitSwitch() {
		return lowerRecycleLimitSwitch;
	}

	public void setLowerRecycleLimitSwitch(boolean lowerRecycleLimitSwitch) {
		this.lowerRecycleLimitSwitch = lowerRecycleLimitSwitch;
	}

	public boolean isUpperToteLimitSwitch() {
		return upperToteLimitSwitch;
	}

	public void setUpperToteLimitSwitch(boolean upperToteLimitSwitch) {
		this.upperToteLimitSwitch = upperToteLimitSwitch;
	}

	public boolean isLowerToteLimitSwitch() {
		return lowerToteLimitSwitch;
	}

	public void setLowerToteLimitSwitch(boolean lowerToteLimitSwitch) {
		this.lowerToteLimitSwitch = lowerToteLimitSwitch;
	}

	public boolean isLeftTouchSensor() {
		return leftTouchSensor;
	}

	public void setLeftTouchSensor(boolean leftTouchSensor) {
		this.leftTouchSensor = leftTouchSensor;
	}

	public boolean isRightTouchSensor() {
		return rightTouchSensor;
	}

	public void setRightTouchSensor(boolean rightTouchSensor) {
		this.rightTouchSensor = rightTouchSensor;
	}

	public boolean isLineSensor() {
		return lineSensor;
	}

	public void setLineSensor(boolean lineSensor) {
		this.lineSensor = lineSensor;
	}

	public boolean isMagnetSensor() {
		return magnetSensor;
	}

	public void setMagnetSensor(boolean magnetSensor) {
		this.magnetSensor = magnetSensor;
	}
	
	public void setActiveIntake(double activeIntakeLeft, double activeIntakeRight ){
	    this.activeMotorIntakeLeft = activeIntakeLeft;
	    this.activeMotorIntakeRight = activeIntakeRight;
	}
	
	public double getActiveIntakeLeft(){
	    return this.activeMotorIntakeLeft;
	}
	
	public double getActiveIntakeRight(){
        return this.activeMotorIntakeRight;
    }
	
	public MotorState getActiveMotorStateLeft(){
	    return this.activeMotorStateLeft;
	}
	
	public MotorState getActiveMotorStateRight(){
        return this.activeMotorStateRight;
    }
	
	public void setActiveMotorState(MotorState left, MotorState right){
	    this.activeMotorStateLeft = left;
	    this.activeMotorStateRight = right;
	}

}
