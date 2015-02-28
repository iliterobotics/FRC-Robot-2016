package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class RobotControl {
	private static RobotControl instance = null;
	private List<Talon> leftDrive;
	private List<Talon> rightDrive;
	private Map<RobotMotorType, Talon> outputTalons;
	private Map<RobotPneumaticType, DoubleSolenoid> outputSolenoids;
	private Map<RobotPneumaticType, Solenoid> solenoid;
	private Map<RobotMotorType, Relay> relays;
	private Compressor compressor;

	/*
	 * private Talon leftDrive1; private Talon leftDrive2; private Talon
	 * rightDrive1; private Talon rightDrive2; private Talon toteMotor; private
	 * Talon recycleBinMotor;
	 * 
	 * //TODO: convert to solenoid private Solenoid grabberPneumatic; private
	 * Solenoid leftShifterPneumatic; private Solenoid rightShifterPneumatic;
	 */
	public static synchronized RobotControl getInstance() {
		if (instance == null) {
			instance = new RobotControl();
		}
		return instance;
	}

	protected RobotControl() {
		compressor = new Compressor();
		compressor.start();
		outputSolenoids = new HashMap<RobotPneumaticType, DoubleSolenoid>();
		solenoid = new HashMap<RobotPneumaticType, Solenoid>();
		outputTalons = new HashMap<RobotMotorType, Talon>();
		relays = new HashMap<RobotMotorType, Relay>();
		rightDrive = new ArrayList<Talon>();
		leftDrive = new ArrayList<Talon>();
	}
	
	public Compressor getCompressor(){
		return compressor;
	}

	public void addTalonOutput(RobotMotorType type, int port) {
		if (type == RobotMotorType.LEFT_DRIVE) {
			leftDrive.add(new Talon(port));
		} else if (type == RobotMotorType.RIGHT_DRIVE) {
			// add to right motor
			rightDrive.add(new Talon(port));
		} else {
			outputTalons.put(type, new Talon(port));
		}
	}

	public List<Talon> getTalons() {
		List<Talon> temp = new ArrayList<Talon>();
		for (RobotMotorType rmt : outputTalons.keySet()) {
			temp.add(outputTalons.get(rmt));
		}
		return temp;
	}
	
	public boolean getSolenoids(RobotPneumaticType rpt){
		return solenoid.get(rpt).get();
	}
	
	public ArrayList[] getSolenoids() {
		ArrayList[] temp = new ArrayList[2];
		ArrayList<Solenoid> tempRegular = new ArrayList<Solenoid>();
		ArrayList<DoubleSolenoid> tempDouble = new ArrayList<DoubleSolenoid>();

		for (RobotPneumaticType rpt : outputSolenoids.keySet())
			tempDouble.add(outputSolenoids.get(rpt));
		for (RobotPneumaticType rpt : solenoid.keySet())
			tempRegular.add(solenoid.get(rpt));
		temp[0] = tempRegular;
		temp[1] = tempDouble;
		return temp;
	}

	public void addRelay(RobotMotorType type, int channel) {
		relays.put(type, new Relay(channel));
	}

	public List<Relay> getRelay() {
		List<Relay> temp = new ArrayList<Relay>();
		for (RobotMotorType rmt : relays.keySet())
			temp.add(relays.get(rmt));
		return temp;
	}

	public void updateRelay(RobotMotorType type, Value state) {
		relays.get(type).set(state);
	}

	public void addPneumaticOutput(RobotPneumaticType type, int port) {
		solenoid.put(type, new Solenoid(port));
	}

	public void addPneumaticOutput(RobotPneumaticType type, int port1, int port2) {
		outputSolenoids.put(type, new DoubleSolenoid(port1, port2));
	}

	public void updateDriveSpeed(double leftspeed, double rightspeed) {
		for (Talon leftMotor : leftDrive) {
			leftMotor.set(-leftspeed);
		}
		for (Talon rightMotor : rightDrive) {
			rightMotor.set(rightspeed);
		}
	}

	public void updateGrabberPneumatics(boolean start) {
		solenoid.get(RobotPneumaticType.GRABBER_PNEUMATIC).set(start);
	}
	
	public void updateWristExtensionPneumatics(DoubleSolenoid.Value start) {
		outputSolenoids.get(RobotPneumaticType.WRIST_EXTENSION).set(start);
	}
	
	public void updateWristRotationPneumatics(DoubleSolenoid.Value start) {
		outputSolenoids.get(RobotPneumaticType.WRIST_ROTATION).set(start);
	}

	public void updateToteStop(boolean start) {
		solenoid.get(RobotPneumaticType.TOTE_LIFT_STOP).set(start);
	}

	public void updateGearShifter(boolean start) {
		solenoid.get(RobotPneumaticType.GEAR_SHIFTER_PNEUMATIC).set(
				start);
	}

	public void updateToteMotor(double speed) {
		outputTalons.get(RobotMotorType.TOTE_LIFT).set(speed);
	}

	public void updateRecycleMotor(double speed) {
		outputTalons.get(RobotMotorType.RECYCLE_LIFT).set(speed);
	}

	public void updateToteSupport(boolean isSupported) {
		solenoid.get(RobotPneumaticType.LIFT_SUPPORT).set(isSupported);
	}
}
