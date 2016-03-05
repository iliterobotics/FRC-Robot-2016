package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShooterTwist extends AutoCommand{

    private final double ANGLE;
    private Shooter shooter;
    
    public AutoShooterTwist(double angle) {
        ANGLE = angle;
        shooter = Shooter.getInstance();
        shooter.reset();
    }
    
    @Override
    public boolean init() {
        shooter.setToTwistValue(ANGLE);
        return true;
    }

    @Override
    public boolean execute() {
//        boolean completed = shooter.positionTwist();
//        DriverStation.reportError("\nPositioning twist:" + completed, false);
        return true;
    }

    @Override
    public boolean updateOutputs() {
        return false;
    }

    @Override
    public void reset() {}

}
