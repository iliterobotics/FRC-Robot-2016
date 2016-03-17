package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShooterInitiate extends AutoCommand{

    public AutoShooterInitiate(){
        
    }
    
    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
        Shooter.getInstance().initiateLaunch();
        Shooter.getInstance().updateOutputs();
        return true;
    }

    @Override
    public boolean updateOutputs() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
