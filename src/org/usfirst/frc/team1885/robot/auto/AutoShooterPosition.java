package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShooterPosition extends AutoCommand {
    private double angle;
    private Shooter shooter;
    public AutoShooterPosition(double angle) {
        this.angle = angle;
        shooter = Shooter.getInstance();
    }
    @Override
    public boolean init() {
        // TODO Auto-generated method stub
        return true;
    }
    @Override
    public boolean execute() {   
        boolean yes = shooter.position(angle);
        shooter.updateOutputs();
        return yes;
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
