package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;
import java.util.List;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import com.kauailabs.nav6.frc.IMU;
import com.sun.javafx.binding.Logging;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTablesJNI.LoggerFunction;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();

    private HashMap<SensorType, LidarSensor> lidar_sensor;
    private IMU imu;
    SerialPort serial_port;

    public static final double DEADZONE = 0.1;

    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    protected SensorInputControlSRX() {
        lidar_sensor = new HashMap<SensorType, LidarSensor>();
    }
    public void setUpNAVX(byte rate,
            edu.wpi.first.wpilibj.SerialPort.Port port) {

        serial_port = new SerialPort(57600, port);
        imu = new IMU(serial_port, rate);
    }
    public IMU getNAVX() {
        return imu;
    }
    public static double deadzone(double axis) {
        if (Math.abs(axis) < DEADZONE) {
            return 0;
        }
        return axis;
    }
    public boolean addLidarSensor(Port port) {
        lidar_sensor.put(SensorType.LIDAR, new LidarSensor(port));
        return true;
    }

    public LidarSensor getLidarSensor(SensorType sensor_type) {
        return lidar_sensor.get(sensor_type);
    }
    public double getCurrent(int channel) {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature() {
        return PDP.getTemperature();
    }

    public void update() {
//        // Lidar Sensor Test
//        StringBuilder output = new StringBuilder();
//        output.append("\n-Lidar Sensor Distance = " + this.getLidarSensor(SensorType.LIDAR).getDistance());
//        DriverStation.reportError(output.toString(), false);
//        Timer.delay(1.0);
        
 //       while(rsrx.getLeftDrive().get(0).)
        
        if(rsrx.getLeftDrive().get(0).getOutputVoltage() <= 1.0 &&  rsrx.getLeftDrive().get(0).getSpeed() > 0){
            DriverStation.reportError("\nVoltage:: " + rsrx.getLeftDrive().get(0).getOutputVoltage(),false);
        }
            
    }
}
