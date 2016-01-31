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
    private Map<Integer, CANTalon> talons;
    private Map<SensorType, CANTalon> sensor;

    public static synchronized RobotControlWithSRX getInstance() {
        if (instance == null) {
            instance = new RobotControlWithSRX();
        }
        return instance;
    }
    protected RobotControlWithSRX() {
        leftDrive = new ArrayList<CANTalon>();
        rightDrive = new ArrayList<CANTalon>();
        talons = new HashMap<Integer, CANTalon>();
        sensor = new HashMap<SensorType, CANTalon>();
    }
    public void addTalonOutput(RobotMotorType type, int port) {
        if (talons.containsKey(port)) {
            if (type == RobotMotorType.LEFT_DRIVE) {
                leftDrive.add(talons.get(port));
            } else if (type == RobotMotorType.RIGHT_DRIVE) {
                rightDrive.add(talons.get(port));
            }
        } else {
            CANTalon ct = new CANTalon(port);
            if (type == RobotMotorType.LEFT_DRIVE) {
                leftDrive.add(ct);
            } else if (type == RobotMotorType.RIGHT_DRIVE) {
                // add to right motor
                rightDrive.add(ct);
            }
            talons.put(port, ct);
        }
    }
    public void addTalonSensor(SensorType sensorType, int port) {
        if(talons.containsKey(port))
        {
            sensor.put(sensorType, talons.get(port));
        }
        else
        {
            CANTalon ct = new CANTalon(port);
            sensor.put(sensorType, ct);
            talons.put(port, ct);
        }
    }
    public void updateDriveSpeed(double leftspeed, double rightspeed) {
        for (CANTalon leftMotor : leftDrive) {
            leftMotor.set(leftspeed);
            // System.out.println(leftMotor.getOutputVoltage() + "Voltage");
        }
        for (CANTalon rightMotor : rightDrive) {
            rightMotor.set(-rightspeed);
        }
    }
    public List<CANTalon> getLeftDrive() {
        return leftDrive;
    }
    public List<CANTalon> getRightDrive() {
        return rightDrive;
    }
    public Map<Integer, CANTalon> getTalons() {
        return this.talons;
    }
    public Map<SensorType, CANTalon> getSensor()
    {
        return this.sensor;
    }
}
