package org.usfirst.frc.team1885.robot.auto;

import java.util.List;

import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoAimTurn extends AutoCommand{
    
    private AutoAlign align;
    private AutoShooterTilt tilt;
    
    public AutoAimTurn(double goalTurn){
        align = new AutoAlign(goalTurn);
        tilt = new AutoShooterTilt(Shooter.HIGH_GOAL_CAM_TILT);
    }
    
    @Override
    public boolean init() {
        (new AutoShooterInitiate()).execute();
        (new AutoAdjustIntake(ActiveIntake.intakeDown)).execute();
        return false;
    }

    @Override
    public boolean execute() {
        tilt.execute();
        return align.execute();
    }

    @Override
    public boolean updateOutputs() {
        align.updateOutputs();
        tilt.updateOutputs();
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
