package org.usfirst.frc.team1885.robot.comms;

import java.util.Arrays;

import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class TelemetryMessage extends Message {

	private static final long serialVersionUID = 4021050113420280510L;
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

	public TelemetryMessage() {
		super(ServerMessageType.TELEMETRY_MSG);
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

	public static long getSerialversionuid() {
		return serialVersionUID;
	}

	private double[] digitalInputs = new double[20];
	private double[] analogInputs = new double[10];
	private double[] relays = new double[4];
	private double[] digitalOutputs = new double[10];
	private double[] solenoids = new double[8];

	public double getSolenoids(int index) {
		if (index >= 0 && index < solenoids.length)
			return solenoids[index];
		else
			return -1;
	}

	public void setSolenoids(int index, double data) {
		if (index >= 0 && index < solenoids.length)
			this.solenoids[index] = data;
	}

	public double getDigitalInputs(int index) {
		if (index >= 0 && index < digitalInputs.length)
			return digitalInputs[index];
		else
			return -1;
	}

	public void setDigitalInputs(int index, double data) {
		if (index >= 0 && index < digitalInputs.length)
			this.digitalInputs[index] = data;
	}

	public double getAnalogInputs(int index) {
		if (index >= 0 && index < analogInputs.length)
			return analogInputs[index];
		else
			return -1;
	}

	public void setAnalogInputs(int index, double data) {
		if (index >= 0 && index < analogInputs.length)
			this.analogInputs[index] = data;
	}

	public double getRelays(int index) {
		if (index >= 0 && index < relays.length)
			return relays[index];
		else
			return -1;
	}

	public void setRelays(int index, double data) {
		if (index >= 0 && index < relays.length)
			this.relays[index] = data;
	}

	public double getDigitalOutputs(int index) {
		if (index >= 0 && index < digitalOutputs.length)
			return digitalOutputs[index];
		else
			return -1;
	}

	public void setDigitalOutputs(int index, double data) {
		if (index >= 0 && index < digitalOutputs.length)
			this.digitalOutputs[index] = data;
	}

	public long getSerialID() {
		return serialVersionUID;
	}

	@Override
	public String toString() {
		return "TelemetryMessage [digitalInputs="
				+ Arrays.toString(digitalInputs) + ", analogInputs="
				+ Arrays.toString(analogInputs) + ", relays="
				+ Arrays.toString(relays) + ", digitalOutputs="
				+ Arrays.toString(digitalOutputs) + ", solenoids="
				+ Arrays.toString(solenoids) + "]";
	}
}
