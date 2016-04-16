package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ActiveIntake;

public class AutoIntakeSetPower extends AutoCommand{

    private double power;
    
    public AutoIntakeSetPower(double power) {
        this.power = power;
    }
    
    @Override
    public boolean init() {
        ActiveIntake.getInstance().setIntakeSpeed(power);
        return true;
    }

    @Override
    public boolean execute() {
        ActiveIntake.getInstance().setIntakeSpeed(power);
        return true;
    }

    @Override
    public boolean updateOutputs() {
        ActiveIntake.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() {}
}
