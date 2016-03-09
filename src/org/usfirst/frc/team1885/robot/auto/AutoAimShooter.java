package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.serverdata.ShooterDataClient;

import dataclient.robotdata.vision.HighGoal;

public class AutoAimShooter extends AutoCommand {

    private static final double HIGHGOAL_MIDPOINT = 97; //inches for midpoint of high goal
    
    private ShooterDataClient sdc;
    private HighGoal hg;
    
    public AutoAimShooter() {
        sdc = ShooterDataClient.startShooterDataClient();
        hg = sdc.getData();
    }
    
    @Override
    public boolean init() {
        double distance = hg.getDistance();
        double azimuth = hg.getAzimuth();
        
        double tiltAngle = 180 - Math.toDegrees(Math.asin(HIGHGOAL_MIDPOINT / distance));
        Shooter.getInstance().setToTiltValue(tiltAngle);
        
        Shooter.getInstance().setToTwistValue(azimuth);        
        
        return true;
    }

    @Override
    public boolean execute() {
        boolean tiltCompleted = Shooter.getInstance().updateTilt();
        boolean twistCompleted = Shooter.getInstance().updateTwist();
        
        return tiltCompleted && twistCompleted;
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
