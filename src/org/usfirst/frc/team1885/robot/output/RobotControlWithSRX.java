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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotControlWithSRX {
    public static RobotControlWithSRX instance;
    private List<CANTalon> leftDrive;
    private List<CANTalon> rightDrive;
    private Map<RobotMotorType, CANTalon> talons = new HashMap<RobotMotorType, CANTalon>();
    private Map<SensorType, CANTalon> sensors;
    private Map<RobotPneumaticType, Solenoid> singleSolenoids;
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
        singleSolenoids = new HashMap<RobotPneumaticType, Solenoid>();
        sensors = new HashMap<SensorType, CANTalon>();
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
        } else {
            CANTalon ct = new CANTalon(port);
            sensors.put(sensorType, ct);
            talons.put(motorType, ct);
        }
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
    public void updateShooterMotors(double shooterSpeedLeft,
            double shooterSpeedRight) {
        talons.get(RobotMotorType.SHOOTER_LEFT).set(shooterSpeedLeft);
        talons.get(RobotMotorType.SHOOTER_RIGHT).set(shooterSpeedRight);
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
    public void updateArmMotors(double jointASpeed, double jointBSpeed) {
        // talons.get(RobotMotorType.ARM_JOINT_A).set(jointASpeed);
        // talons.get(RobotMotorType.ARM_JOINT_B).set(jointBSpeed);
    }
    public void addSingleSolenoid(RobotPneumaticType type, int port) {
        singleSolenoids.put(type, new Solenoid(port));
    }
    public void updateSingleSolenoid(RobotPneumaticType type, boolean value) {
        singleSolenoids.get(type).set(value);
    }
    public Solenoid getSingleSolenoid(RobotPneumaticType type) {
        return singleSolenoids.get(type);
    }
    public Map<SensorType, CANTalon> getSensor() {
        DriverStation.reportError(sensors == null ? "No sensor" : "Sensors!",
                false);
        return this.sensors;
    }
}
