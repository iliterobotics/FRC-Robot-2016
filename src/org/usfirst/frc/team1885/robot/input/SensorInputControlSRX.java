package org.usfirst.frc.team1885.robot.input;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;
import org.usfirst.frc.team1885.robot.sensor.PressureSensor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    private LidarSensor ls;
    private BuiltInAccelerometer bia;
    private AHRS navx;
    private PressureSensor ps;

    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public void update() {
        StringBuilder output = new StringBuilder();
        // output.append("\n-LIDAR SENSOR DISTANCE= "
        // + this.getLidarSensor().getDistance());
        this.getLidarSensor().update();
        output.append("\n-Potentiometer In Position: "
                + ((this.getAnalogInPosition(SensorType.ULTRASONIC) / 1024.0)
                        * 360));
        output.append("sd\n-Potentiometer Generic: "
                + (this.getAnalogGeneric(SensorType.ULTRASONIC)));
        output.append(
                "\n-Lidar value:" + (this.getLidarSensor().getDistance()));

        DriverStation.reportError(output.toString(), false);
        Timer.delay(1);
        // System.out.println(this.bia.getX() +" x " + bia.getY() + " y " +
        // bia.getZ() + " z ");
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
    public void addPressureSensor(int channel) {
        ps = new PressureSensor(channel);
    }
    public double getPressureAverageVoltage() {
        return ps.getAverageVoltage();
    }
    public double getPressureVoltage() {
        return ps.getVoltage();
    }

    /**
     * Private constructor because this is a singleton!!
     */
    protected SensorInputControlSRX() {

    }

}
