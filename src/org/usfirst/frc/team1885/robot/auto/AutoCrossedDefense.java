package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

public class AutoCrossedDefense extends AutoCommand {
    // autonomous checkpoint used to check anything after we cross a defense and
    // when to stop

    private final double FLAT_STANDARD = 2;
    
    private SensorInputControlSRX sensorInputControl = SensorInputControlSRX
            .getInstance();
    private final double WAIT_TIME = 0.5;
    private long startTime;

    @Override
    public boolean init() {
        startTime = System.currentTimeMillis();
        return true;
    }

    @Override
    public boolean execute() {
        if(sensorInputControl.getNavX().getRoll() <= FLAT_STANDARD && sensorInputControl.getNavX().getPitch() <= FLAT_STANDARD){
            if(System.currentTimeMillis() - startTime > WAIT_TIME){
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // no outputs to update, only a checkpoint
        return false;
    }

    @Override
    public void reset() {
        // no outputs changed, only a checkpoint
    }

}
