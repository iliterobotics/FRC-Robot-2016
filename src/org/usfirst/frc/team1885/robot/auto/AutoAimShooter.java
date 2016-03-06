package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.serverdata.ShooterDataClient;

import dataclient.robotdata.vision.HighGoal;

public class AutoAimShooter extends AutoCommand {

    private static final double HIGHGOAL_MIDPOINT = 97; //inches for midpoint of high goal
    
    private Shooter shooter;
    private ShooterDataClient sdc;
    private HighGoal hg;
    
    public AutoAimShooter() {
        shooter = Shooter.getInstance();
        shooter.reset();
        
        sdc = ShooterDataClient.startShooterDataClient();
        hg = sdc.getData();
    }
    
    @Override
    public boolean init() {
        double distance = hg.getDistance();
        double azimuth = hg.getAngleOfElevation();
        
        double tiltAngle = 180 - Math.toDegrees(Math.asin(HIGHGOAL_MIDPOINT / distance));
        shooter.setToTiltValue(tiltAngle);
        
        shooter.setToTwistValue(azimuth);        
        
        return true;
    }

    @Override
    public boolean execute() {
        boolean tiltCompleted = shooter.updateTiltPosition();
        boolean twistCompleted = shooter.updateTwistPosition();
        
        return tiltCompleted && twistCompleted;
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
