package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoShooterTwist extends AutoCommand {
    private static final double ERROR = 0.5;
    private final double angle;
    private double currentAngle;    
    
    public AutoShooterTwist(double angle) {
        this.angle = angle;
    }
    @Override
    public boolean init() {
        currentAngle = Shooter.getInstance().getRelativeTwist();
        return true;
    }
    @Override
    public boolean execute() {
        currentAngle = Shooter.getInstance().getRelativeTwist();
        if(Math.abs(currentAngle - angle) <= ERROR){
            return true;
        }
        if(currentAngle < angle)
            currentAngle += Shooter.TWIST_MOVEMENT_PROPORTION;
        else
           currentAngle -= Shooter.TWIST_MOVEMENT_PROPORTION;
        Shooter.getInstance().setToTwistValue(currentAngle);
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
