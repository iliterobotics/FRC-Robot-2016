package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SensorInputControlSRX {

    private static final int TICKS_IN_360 = 1024;
    private double INITIAL_PITCH; // Shouldn't change
    private double INITIAL_ROLL; // Shouldn't change
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx;
    private PowerDistributionPanel PDP;
    private LidarSensor ls;
    private BuiltInAccelerometer bia;
    private AHRS navx;

    public static final double DEADZONE = 0.1;

    public double INITIAL_POT_B_POSITION;
    public double INITIAL_POT_A_POSITION;
    public static double INITIAL_TILT_POSITION;
    private static final double POTENTIOMETER_CONVERSION_FACTOR = 1024 / 360;
    private Map<SensorType, Integer> ticks;

    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    private SensorInputControlSRX() {
        rsrx = RobotControlWithSRX.getInstance();
        PDP = new PowerDistributionPanel();
        ticks = new HashMap<SensorType, Integer>();
    }
    public void update() {
        // StringBuilder output = new StringBuilder();
        // output.append("\nLeft Flywheel Velocity: " +
        // getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER));
        // output.append("\nRight Flywheel Velocity: " +
        // getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER));
        // output.append("\nTilt Potentiometer: " +
        // getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER));
        // output.append("\n Twist Position: " +
        // getEncoderAbsolutePosition(SensorType.SHOOTER_TWIST_ENCODER));
        // DriverStation.reportError(output + "\n", false);
        /*
         * Encoder values testing
         * 
         * DriverStation.reportError("\nRight Encoder Value::" +
         * getEncoderDistance(SensorType.RIGHT_ENCODER) +
         * " --- Left Encoder Value:: " +
         * getEncoderDistance(SensorType.LEFT_ENCODER), false);
         */
    }
    public double getInitPitch() {
        return INITIAL_PITCH;
    }
    public double getInitRoll() {
        return INITIAL_ROLL;
    }
    public double getPitch() {
        return navx.getPitch();
    }
    public double getYaw() {
        return navx.getYaw();
    }
    public double getRoll() {
        return navx.getRoll();
    }
    public void init() {
        INITIAL_POT_A_POSITION = rsrx.getSensor()
                .get(SensorType.JOINT_A_CTRE_ABSOLUTE).getEncPosition();
        INITIAL_POT_B_POSITION = rsrx.getSensor()
                .get(SensorType.JOINT_B_CTRE_ABSOLUTE).getEncPosition();
        DriverStation.reportError("\nINIT_POT_A: " + INITIAL_POT_A_POSITION
                + " --- INIT_POT_B: " + INITIAL_POT_B_POSITION, false);
    }
    public double getCurrent(int channel) {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature() {
        return PDP.getTemperature();
    }
    public double getAnalogInPosition(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInPosition();
    }

    public void setEncoderPosition(SensorType type, int pos) {
        rsrx.getSensor().get(type).setEncPosition(pos);
    }
    public double getAnalogGeneric(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInRaw();
    }
    public double getZeroedPotentiometer(SensorType type) {
        switch (type) {
        case JOINT_A_CTRE_ABSOLUTE:
            return getAnalogGeneric(type) - INITIAL_POT_A_POSITION;
        case JOINT_B_CTRE_ABSOLUTE:
            return getAnalogGeneric(type) - INITIAL_POT_B_POSITION;
        default:
            return getAnalogGeneric(type);
        }
    }
    public boolean digitalLimitSwitch(SensorType type) {
        return rsrx.getSensor().get(type).isFwdLimitSwitchClosed();
    }
    public int getEncoderPos(SensorType type) {
        return rsrx.getSensor().get(type).getEncPosition();
    }

    public int getEncoderVelocity(SensorType type) {
        return rsrx.getSensor().get(type).getEncVelocity();
    }
    public void addPotentiometer(RobotMotorType motorType,
            SensorType sensorType, int port) {
        rsrx.addTalonSensor(motorType, sensorType, port);
    }
    public void addEncoder(RobotMotorType motorType, SensorType sensorType,
            int port) {
        rsrx.addTalonSensor(motorType, sensorType, port);
    }
    public double getEncoderAbsolutePosition(SensorType type) {
        return rsrx.getSensor().get(type).getPulseWidthPosition();
    }

    public void addLidarSensor(Port port) {
        ls = new LidarSensor(port);
    }
    public LidarSensor getLidarSensor() {
        return this.ls;

    }
    public void createAccelerometer() {
        bia = new BuiltInAccelerometer();
    }
    public BuiltInAccelerometer getAccelerometer() {
        return bia;
    }
    public void createNavX(SerialPort.Port port) {
        navx = new AHRS(port);
    }
    public AHRS getNavX() {
        return navx;
    }
    public void calibrateGyro() {
        navx.zeroYaw();
        Timer.delay(.3); // Time to calibrate gyro
        INITIAL_PITCH = navx.getPitch();
        INITIAL_ROLL = navx.getRoll();
    }
    public double getInitialPotAPostition() {
        return INITIAL_POT_A_POSITION;
    }
    public double getInitialPotBPostition() {
        return INITIAL_POT_B_POSITION;
    }
    public void resetEncoder(SensorType type) {
        rsrx.getSensor().get(type).setEncPosition(0);
    }
}