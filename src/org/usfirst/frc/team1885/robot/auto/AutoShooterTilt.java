package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShooterTilt extends AutoCommand {
    private static final double ERROR = 0.5;
    private final double angle;
    private double currentAngle;
    private Shooter shooter;
    
    public AutoShooterTilt(double angle) {
        this.angle = angle;
        shooter = Shooter.getInstance();
        shooter.reset();
    }
    @Override
    public boolean init() {
        return true;
    }
    @Override
    public boolean execute() {
        if(Math.abs(currentAngle - angle) <= ERROR){
            return true;
        }
        currentAngle += Shooter.TILT_MOVEMENT_PROPORTION;
        return false;
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
