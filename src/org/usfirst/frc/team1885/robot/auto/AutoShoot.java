package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShoot extends AutoCommand {
   
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
        return Shooter.getInstance().fire();
    }

    @Override
    public boolean updateOutputs() {
        Shooter.getInstance().update();
        Shooter.getInstance().updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
