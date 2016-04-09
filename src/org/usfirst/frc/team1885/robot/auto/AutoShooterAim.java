package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.UtilityArm;

public class AutoShooterAim extends AutoCommand {
    
    private final long WAIT_TIME = 500;
    private AutoShooterTilt shooterTilt;
    private AutoShooterTwist shooterTwist;
    private long aimDurationStart;
    
    public AutoShooterAim() {
        shooterTilt = new AutoShooterTilt(Shooter.HIGH_GOAL_CAM_TILT);
        shooterTwist = new AutoShooterTwist(0);
    }
    
    @Override
    public boolean init()  {
        aimDurationStart = System.currentTimeMillis();
        return true;
    }

    @Override
    public boolean execute() {
        double tilt = Shooter.getInstance().getTiltAimLock();
        double twist = Shooter.getInstance().getTwistAimLock();
        
        shooterTilt = new AutoShooterTilt(tilt);
        shooterTwist = new AutoShooterTwist(twist);
        
        shooterTilt.execute();
        shooterTwist.execute();
        if(Shooter.getInstance().isAimed()){
            return true;
        }
        if(timeOut()){
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        UtilityArm.getInstance().update();
        UtilityArm.getInstance().updateOutputs();
        shooterTwist.updateOutputs();
        shooterTilt.updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

}
