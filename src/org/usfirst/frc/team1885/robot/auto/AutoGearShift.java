package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class AutoGearShift extends AutoCommand{

    private boolean gear;
    
    public AutoGearShift(boolean gear){
        this.gear = gear;
    }
    
    @Override
    public boolean init() {
        DrivetrainControl.getInstance().gearShift(gear);
        return true;
    }

    @Override
    public boolean execute() { return true; }

    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() { }

}
