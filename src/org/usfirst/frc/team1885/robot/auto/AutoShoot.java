package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShoot extends AutoCommand {
   
    private boolean shooting;
    
    public AutoShoot(boolean shooting){
        this.shooting = shooting;
    }
    
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
        if(!shooting){
            DriverStation.reportError("\nStopping shooter flywheels", false);
            Shooter.getInstance().setFlywheels(0);
            if(timeOut()){
                return true;
            }
            return false;
        }
        return Shooter.getInstance().fire();
    }

    @Override
    public boolean updateOutputs() {
        Shooter.getInstance().updateTilt();
        Shooter.getInstance().updateTwist();
        Shooter.getInstance().updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
