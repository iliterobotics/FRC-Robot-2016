package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShooterShoot extends AutoCommand {
    private Shooter shooter;
   
    public boolean init() {
        shooter = shooter.getInstance();
        shooter.reset();
        return false;
    }

    @Override
    public boolean execute() {
        return true;
        // return shooter.flywheelOutSpeed();
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
