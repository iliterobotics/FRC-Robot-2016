package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoShooterTwist extends AutoCommand {
    
    private final double angle;
    
    public AutoShooterTwist(double angle) {
        this.angle = angle;
    }
    @Override
    public boolean init() {
        return true;
    }
    @Override
    public boolean execute() {
        Shooter.getInstance().setToTwistValue(this.angle);
        return true;
    }
    @Override
    public boolean updateOutputs() {
        ModuleControl.getInstance().updateIntakeShooter();
        ModuleControl.getInstance().updateIntakeShooterOutputs();
        return false;
    }
    @Override
    public void reset() {}
}
