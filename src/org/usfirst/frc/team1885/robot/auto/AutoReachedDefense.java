package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

public class AutoReachedDefense extends AutoCommand {
    // autonomous checkpoint used to check anything before we reach a defense

    private SensorInputControlSRX sensorInputControl = SensorInputControlSRX
            .getInstance();

    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
        if(sensorInputControl.getNavX().getRoll() >= AutonomousRoutine.PITCH_CHANGE){
            return true;
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
