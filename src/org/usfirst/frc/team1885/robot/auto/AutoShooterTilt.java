package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShooterTilt extends AutoCommand {
    private final double angle;
    private Shooter shooter;
    
    public AutoShooterTilt(double angle) {
        this.angle = angle;
        shooter = Shooter.getInstance();
        shooter.reset();
    }
    @Override
    public boolean init() {
        shooter.setToTiltValue(angle);
        return true;
    }
    @Override
    public boolean execute() {
        boolean completed = shooter.positionTilt();
        DriverStation.reportError("\nPositioning Shootr..." + completed, false);
        return completed;
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
