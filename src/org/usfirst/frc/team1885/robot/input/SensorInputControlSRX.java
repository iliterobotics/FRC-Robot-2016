package org.usfirst.frc.team1885.robot.input;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public double getCurrent(int channel)
    {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature()
    {
        return PDP.getTemperature();
    }    
}
