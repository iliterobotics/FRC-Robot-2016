package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.UtilityArmPosition;
import org.usfirst.frc.team1885.robot.modules.UtilityArm;

public class AutoMoveUtilityArm extends AutoCommand{

    private UtilityArmPosition position;
    
    public AutoMoveUtilityArm(UtilityArmPosition position) {
        this.position = position;
    }
    
    @Override
    public boolean init() {
        UtilityArm.getInstance().setPosition(position);
        return true;
    }

    @Override
    public boolean execute() {
        UtilityArm.getInstance().update();
        return UtilityArm.getInstance().setPosition(position);
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
