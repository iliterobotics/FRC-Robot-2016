package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotControlWithSRX {
    public static RobotControlWithSRX instance;
    private List<CANTalon> leftDrive;
    private List<CANTalon> rightDrive;
    private Map<RobotMotorType, CANTalon> talons;
    private Map<SensorType, CANTalon> sensors;
    private Map<RobotPneumaticType, Solenoid> singleSolenoids;
    private Map<RobotPneumaticType, DoubleSolenoid> doubleSolenoids;
    private Compressor c;

    public static synchronized RobotControlWithSRX getInstance() {
        if (instance == null) {
            instance = new RobotControlWithSRX();
        }
        return instance;
    }
    protected RobotControlWithSRX() {
        c = new Compressor(0);
        c.start();
        leftDrive = new ArrayList<CANTalon>();
        rightDrive = new ArrayList<CANTalon>();
        talons = new HashMap<RobotMotorType, CANTalon>();
        sensors = new HashMap<SensorType, CANTalon>();
        singleSolenoids = new HashMap<RobotPneumaticType, Solenoid>();
        doubleSolenoids = new HashMap<RobotPneumaticType, DoubleSolenoid>();
    }
    public void addTalonOutput(RobotMotorType type, int port) {
        if (type == RobotMotorType.LEFT_DRIVE) {
            leftDrive.add(new CANTalon(port));
        } else if (type == RobotMotorType.RIGHT_DRIVE) {
            // add to right motor
            rightDrive.add(new CANTalon(port));
        } else {
            talons.put(type, new CANTalon(port));
        }
    }
    public void addTalonSensor(RobotMotorType motorType, SensorType sensorType,
            int port) {
        if (talons.containsKey(motorType)) {
            sensors.put(sensorType, talons.get(motorType));
        }else{
            CANTalon talon = new CANTalon(port);
            talons.put(motorType, talon);
            sensors.put(sensorType, talon);
        }
    }
    public void addDoubleSolenoid(RobotPneumaticType type, int port1,
            int port2) {
        doubleSolenoids.put(type, new DoubleSolenoid(port1, port2));
    }
    public Compressor getCompressor() {
        return c;
    }
    public void updateDriveSpeed(double leftspeed, double rightspeed) {
        for (CANTalon leftMotor : leftDrive) {
            leftMotor.set(-leftspeed);
            // System.out.println(leftMotor.getOutputVoltage() + "Voltage");
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

    public List<CANTalon> getLeftDrive() {
        return leftDrive;
    }
    public List<CANTalon> getRightDrive() {
        return rightDrive;
    }
    public Map<RobotMotorType, CANTalon> getTalons() {
        return this.talons;
    }
    public void updateDoubleSolenoid(RobotPneumaticType type,
            DoubleSolenoid.Value value) {
        doubleSolenoids.get(type).set(value);
    }
    public void updateIntakeMotors(double intakeLeftSpeed,
            double intakeRightSpeed) {
        // TODO Auto-generated method stub
    }
    public void updateArmMotors(double jointASpeed, double jointBSpeed) {
        // talons.get(RobotMotorType.ARM_JOINT_A).set(jointASpeed);
        // talons.get(RobotMotorType.ARM_JOINT_B).set(jointBSpeed);
    }
    public void gearShift(Value gear) {
        doubleSolenoids.get(RobotPneumaticType.GEAR_SHIFT).set(gear);
        // May need to be fixed, not sure if it is correct.
    }
    public void addSingleSolenoid(RobotPneumaticType type, int port) {
        singleSolenoids.put(type, new Solenoid(port));
    }
    public void addDoubleSolenoid(RobotPneumaticType type, int port) {
        doubleSolenoids.put(type, new DoubleSolenoid(port, port + 1));
    }
    public void updateSingleSolenoid(RobotPneumaticType type, boolean value) {
        singleSolenoids.get(type).set(value);
    }
    public Solenoid getSingleSolenoid(RobotPneumaticType type) {
        return singleSolenoids.get(type);
    }
    public DoubleSolenoid getDoubleSolenoid(RobotPneumaticType type ) {
        return doubleSolenoids.get(type);
    }
    public Map<SensorType, CANTalon> getSensor() {
        return this.sensors;
    }
}
