package org.usfirst.frc.team1885.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

public class RotarySwitchSensor extends AnalogInput{

    public RotarySwitchSensor(int channel) {
        super(channel);
    }
    public double getPosition(){
        return getVoltage();
    }

}
