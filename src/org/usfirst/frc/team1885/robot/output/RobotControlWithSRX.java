package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;

import edu.wpi.first.wpilibj.CANTalon;

public class RobotControlWithSRX {
    public static RobotControlWithSRX instance;
    private List<CANTalon> leftDrive;
    private List<CANTalon> rightDrive;
    private Map<RobotMotorType, CANTalon> talons = new HashMap<RobotMotorType, CANTalon>();
    private Map<SensorType, CANTalon> sensors;

    public static synchronized RobotControlWithSRX getInstance() {
        if (instance == null) {
            instance = new RobotControlWithSRX();
        }
        return instance;
    }
    protected RobotControlWithSRX() {
        leftDrive = new ArrayList<CANTalon>();
        rightDrive = new ArrayList<CANTalon>();
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
    public void updateDriveSpeed(double leftspeed, double rightspeed) {
        for (CANTalon leftMotor : leftDrive) {
            leftMotor.set(-leftspeed);
            // System.out.println(leftMotor.getOutputVoltage() + "Voltage");
        }
        for (CANTalon rightMotor : rightDrive) {
            rightMotor.set(rightspeed);
        }
    }
    public void updateIntakeMotors(double intakeSpeed) {
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
    public void updateIntakeMotors(double intakeLeftSpeed,
            double intakeRightSpeed) {
        // TODO Auto-generated method stub

    }
    public void updateArmMotors(double jointASpeed, double jointBSpeed) {
        talons.get(RobotMotorType.ARM_JOINT_A).set(jointASpeed);
        talons.get(RobotMotorType.ARM_JOINT_B).set(jointBSpeed);
    }

    public Map<SensorType, CANTalon> getSensor() {
        return this.sensors;
    }
}
