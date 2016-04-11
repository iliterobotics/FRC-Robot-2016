package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.UtilityArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

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
        return Math.abs(Shooter.getInstance().getTilt() - this.angle) < Shooter.TILT_ERROR;
    }
    @Override
    public boolean updateOutputs() {
        UtilityArm.getInstance().update();
        UtilityArm.getInstance().updateOutputs();
        ModuleControl.getInstance().updateIntakeShooter();
        ModuleControl.getInstance().updateIntakeShooterOutputs();
        return false;
    }
    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
