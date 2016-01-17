package org.usfirst.frc.team1885.robot.input;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    private LidarSensor ls;
    private BuiltInAccelerometer bia;
    
    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public void update()
    {
        System.out.println(this.getLidarSensor().getDistance());
        System.out.println(this.analogLimitSwitch(SensorType.LIMIT_SWITCH));
        System.out.println(this.bia.getX() +" x " + bia.getY() + " y " + bia.getZ() + " z ");
    }
    public double getCurrent(int channel)
    {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature()
    {
        return PDP.getTemperature();
    }
    public int analogLimitSwitch(SensorType type)
    {
        return rsrx.getSensor().get(type).getAnalogInPosition();
    }
    public boolean digitalLimitSwitch(SensorType type)
    {
        return rsrx.getSensor().get(type).isFwdLimitSwitchClosed();
    }
    public int getEncoderPos(SensorType type)
    {
        return rsrx.getSensor().get(type).getEncPosition();
    }
    public int getEncoderVelocity(SensorType type)
    {
        return rsrx.getSensor().get(type).getEncVelocity();
    }
    public void addLidarSensor(Port port)
    {
        ls = new LidarSensor(port);
    }
    public LidarSensor getLidarSensor()
    {
        return this.ls;
    }
    public void createAccelerometer()
    {
        bia = new BuiltInAccelerometer();
    }
    public void getAccelerometer()
    {
        return bia;
    }
    
}
