package org.usfirst.frc.team1885.robot.input;

import java.util.List;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    
    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public void update()
    {
        System.out.println(this.getEncoderPos(SensorType.DRIVE_TRAIN_ENCODER));
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
}
