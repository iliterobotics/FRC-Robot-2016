package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShooterShoot extends AutoCommand {
    private Shooter shooter;
   
    public boolean init() {
        shooter = shooter.getInstance();
        return false;
    }

    @Override
    public boolean execute() {
        if(shooter.fire())
            return true;
        return false;
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
