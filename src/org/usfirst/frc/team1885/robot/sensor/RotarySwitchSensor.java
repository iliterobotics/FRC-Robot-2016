package org.usfirst.frc.team1885.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class RotarySwitchSensor extends AnalogInput{

    private static final double FIRST_POSITION = 0.004882812034338713;
    
    public RotarySwitchSensor(int channel) {
        super(channel);
    }
    public double getPosition(){
        return ((int)((getVoltage() - FIRST_POSITION) * 2)) * 1.0;
    }
    
}
