package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RelayType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotControlWithSRX {
    
    public static RobotControlWithSRX instance;
    private List<CANTalon> leftDrive;
    private List<CANTalon> rightDrive;
    private Map<RobotMotorType, CANTalon> talons;
    private Map<SensorType, CANTalon> sensors;
    private Map<RobotPneumaticType, Solenoid> singleSolenoids;
    private Map<RobotPneumaticType, DoubleSolenoid> doubleSolenoids;
    private Map<RelayType, Relay> relays;
    private Compressor c;

    public static synchronized RobotControlWithSRX getInstance() {
        if (instance == null) {
            instance = new RobotControlWithSRX();
        }
        return instance;
    }
    private RobotControlWithSRX() {
        c = new Compressor(0);
        c.start();
        leftDrive = new ArrayList<CANTalon>();
        rightDrive = new ArrayList<CANTalon>();
        talons = new HashMap<RobotMotorType, CANTalon>();
        sensors = new HashMap<SensorType, CANTalon>();
        singleSolenoids = new HashMap<RobotPneumaticType, Solenoid>();
        doubleSolenoids = new HashMap<RobotPneumaticType, DoubleSolenoid>();
        relays = new HashMap<RelayType, Relay>();
    }
    
    //Methods that add outputs
    public void addTalonOutput(RobotMotorType type, int port) {
        CANTalon talon = new CANTalon(port);
        if (type == RobotMotorType.LEFT_DRIVE) {
            leftDrive.add(talon);
            if (port != 1) {
                talon.changeControlMode(TalonControlMode.Follower);
                talon.set(1);
            } else {
                talons.put(type, talon);
            }
        } else if (type == RobotMotorType.RIGHT_DRIVE) {
            rightDrive.add(talon);
            if (port != 2) {
                talon.changeControlMode(TalonControlMode.Follower);
                talon.set(2);
            } else {
                talons.put(type, talon);
            }
        } else {
            talons.put(type, talon);
        }
    }
    public void addTalonSensor(RobotMotorType motorType, SensorType sensorType,
            int port) {
        if (talons.containsKey(motorType)) {
            sensors.put(sensorType, talons.get(motorType));
        } else {
            CANTalon talon = new CANTalon(port);
            talons.put(motorType, talon);
            sensors.put(sensorType, talon);
        }
    }
    public void addSingleSolenoid(RobotPneumaticType type, int port) {
        singleSolenoids.put(type, new Solenoid(port));
    }
    public void addDoubleSolenoid(RobotPneumaticType type, int port) {
        doubleSolenoids.put(type, new DoubleSolenoid(port, port + 1));
    }
    public void addDoubleSolenoid(RobotPneumaticType type, int port1,
            int port2) {
        doubleSolenoids.put(type, new DoubleSolenoid(port1, port2));
    }
    public Compressor getCompressor() {
        return c;
    }
    public void addRelay(RelayType type, int channel){
        relays.put(type, new Relay(channel));
    }
    
    //Functions that are used to update components
    public Map<RobotMotorType, CANTalon> getTalons() {
        return this.talons;
    }
    public Map<SensorType, CANTalon> getSensor() {
        return this.sensors;
    }
    public Map<RelayType, Relay> getRelays(){
        return this.relays;
    }
    public List<CANTalon> getLeftDrive() {
        return leftDrive;
    }
    public List<CANTalon> getRightDrive() {
        return rightDrive;
    }
    public void updateSingleSolenoid(RobotPneumaticType type, boolean value) {
        singleSolenoids.get(type).set(value);
    }
    public void updateDoubleSolenoid(RobotPneumaticType type, Value state) {
        doubleSolenoids.get(type).set(state);
    }
    
    //Functions that modules use to update
    public void updateDriveSpeed(double leftspeed, double rightspeed) {
        for (CANTalon leftMotor : leftDrive) {
            leftMotor.set(-leftspeed);
        }
        for (CANTalon rightMotor : rightDrive) {
            rightMotor.set(rightspeed);
        }
    }
    public void updateIntakeMotor(double intakeSpeed) {
        talons.get(RobotMotorType.ACTIVE_INTAKE).set(intakeSpeed);
    }
    public void updateShooterTilt(double tiltSpeed) {
        talons.get(RobotMotorType.SHOOTER_TILT).set(tiltSpeed);
    }
    public void updateShooterTwist(double twistSpeed) {
        talons.get(RobotMotorType.SHOOTER_TWIST).set(twistSpeed);
    }
    public void updateFlywheelShooter(double flywheelSpeedLeft,
            double flywheelSpeedRight) {
        talons.get(RobotMotorType.FLYWHEEL_LEFT).set(flywheelSpeedLeft);
        talons.get(RobotMotorType.FLYWHEEL_RIGHT).set(flywheelSpeedRight);
    }
    public void updateIntakeMotors(double intakeLeftSpeed,
            double intakeRightSpeed) {
    }
    public void gearShift(boolean gear) {
        singleSolenoids.get(RobotPneumaticType.GEAR_SHIFT).set(gear);
    }
    public void updateArmMotors(double jointAPosition, double jointBPosition) {
        talons.get(RobotMotorType.ARM_JOINT_A).set(jointAPosition);
        talons.get(RobotMotorType.ARM_JOINT_B).set(jointBPosition);
    }
    
}
