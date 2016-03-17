package org.usfirst.frc.team1885.robot.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
/**
 * Returns true if received transmission, false if did not receive transmission.
 * @author iliterobotics
 *  
 */
public class BeamSensor extends DigitalInput {
    
    public BeamSensor(int channel) {
        super(channel);
    }
    
}
