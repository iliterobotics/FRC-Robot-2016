package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.serverdata.ShooterDataClient;

import dataclient.robotdata.vision.HighGoal;

public class AutoShooterAim extends AutoCommand {
    
    private long startTime;
    private AutoShooterTilt shooterTilt;
    private AutoShooterTwist shooterTwist;
    
    public AutoShooterAim() {
        shooterTilt = new AutoShooterTilt(Shooter.HIGH_GOAL_CAM_TILT);
        shooterTwist = new AutoShooterTwist(0);
    }
    
    @Override
    public boolean init()  {
        startTime = System.currentTimeMillis();
        return true;
    }

    @Override
    public boolean execute() {
        if(Shooter.getInstance().isGoalFound()){
            
            double tilt = Shooter.getInstance().getTiltAimLock();
            double twist = Shooter.getInstance().getTwistAimLock();
        
            shooterTilt = new AutoShooterTilt(tilt);
            shooterTwist = new AutoShooterTwist(twist);
        
            return shooterTilt.execute() && shooterTwist.execute();
        }
        return System.currentTimeMillis() - startTime > TIMEOUT;
    }

    @Override
    public boolean updateOutputs() {
        shooterTwist.updateOutputs();
        shooterTilt.updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

}
