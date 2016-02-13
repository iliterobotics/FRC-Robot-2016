package org.usfirst.frc.team1885.robot.input;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SensorInputControlSRX {
    private double INITIAL_PITCH; // Shouldn't change
    private double INITIAL_ROLL; // Shouldn't change
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    private LidarSensor ls;
    private BuiltInAccelerometer bia;
    private AHRS navx;

    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public void update() {
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
    public double getCurrent(int channel) {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature() {
        return PDP.getTemperature();
    }
    public double getAnalogInPosition(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInPosition();
    }

    public double getAnalogGeneric(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInRaw();
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

    public double getEncoderDistance(SensorType type) {
        return RobotConfiguration.WHEEL_DIAMETER * Math.PI * getEncoderPos(type)
                * 360.0 / 1440.0;
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

    /**
     * Private constructor because this is a singleton!!
     */
    private SensorInputControlSRX() {

    }

}