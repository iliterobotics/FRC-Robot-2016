package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.UtilityArm;

public class AutoMoveUtilityArm extends AutoCommand{

    private double power;
    
    public AutoMoveUtilityArm(double power) {
        this.power = power;
    }
    
    @Override
    public boolean init() {
        UtilityArm.getInstance().setPower(this.power);
        return true;
    }

    @Override
    public boolean execute() {
        UtilityArm.getInstance().update();
        return true;
    }

    @Override
    public boolean updateOutputs() {
        UtilityArm.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
