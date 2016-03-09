package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShoot extends AutoCommand {
   
    public boolean init() {
        return false;
    }

    @Override
    public boolean execute() {
        if(Shooter.getInstance().fire())
            return true;
        return false;
    }

    @Override
    public boolean updateOutputs() {
        Shooter.getInstance().updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
