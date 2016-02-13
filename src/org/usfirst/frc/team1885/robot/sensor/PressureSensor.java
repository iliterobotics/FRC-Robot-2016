package org.usfirst.frc.team1885.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensor extends AnalogInput {

    public static final double battery_voltage = 5.0;
    public PressureSensor(int channel) {
        super(channel);
    }
    public double getPressure()
    {
        return (250.0 * (getVoltage()/battery_voltage)) - 25;
    }

}

