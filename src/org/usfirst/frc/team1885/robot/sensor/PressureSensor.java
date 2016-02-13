package org.usfirst.frc.team1885.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensor extends AnalogInput {
    
    public static final int NORMALIZED_PRESSURE = 120;
    public static final double BATTERY_VOLTAGE = 5.0;
    public PressureSensor(int channel) {
        super(channel);
    }
    public double getPressure()
    {
        return (250.0 * (getVoltage()/BATTERY_VOLTAGE)) - 25;
    }
}
