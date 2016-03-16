package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.serverdata.ShooterDataClient;

import dataclient.robotdata.vision.HighGoal;

public class AutoShooterAim extends AutoCommand {
    
    private AutoShooterTilt shooterTilt;
    private AutoShooterTwist shooterTwist;
    private AutoDriveStart backup;
    
    public AutoShooterAim() {
        shooterTilt = new AutoShooterTilt(Shooter.HIGH_GOAL_CAM_TILT);
        shooterTwist = new AutoShooterTwist(0);
        backup = new AutoDriveStart(0);
    }
    
    @Override
    public boolean init()  {
        return true;
    }

    @Override
    public boolean execute() {
        if(Shooter.getInstance().isGoalFound()){
            backup = new AutoDriveStart(0);
            
            double tilt = Shooter.getInstance().getTiltAimLock();
            double twist = Shooter.getInstance().getTwistAimLock();
        
            shooterTilt = new AutoShooterTilt(tilt);
            shooterTwist = new AutoShooterTwist(twist);
        
            return shooterTilt.execute() && shooterTwist.execute();
        } else{
            backup = new AutoDriveStart(0.1);
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        shooterTwist.updateOutputs();
        shooterTilt.updateOutputs();
        backup.updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

}
