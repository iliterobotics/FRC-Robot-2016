package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShooterTilt extends AutoCommand {
    private final double angle;
    public AutoShooterTilt(double angle) {
        this.angle = angle;
    }
    @Override
    public boolean init() {
        return true;
    }
    @Override
    public boolean execute() {
        Shooter.getInstance().setToTiltValue(this.angle);
        return false;
    }
    @Override
    public boolean updateOutputs() {
        ModuleControl.getInstance().updateIntakeShooter();
        ModuleControl.getInstance().updateIntakeShooterOutputs();
        return false;
    }
    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
