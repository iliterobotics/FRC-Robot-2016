package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoAdjustShooterTooth extends AutoCommand{

    private boolean toothState;
    
    public AutoAdjustShooterTooth(boolean toothState) {
        this.toothState = toothState;
    }
    
    @Override
    public boolean init() {
        Shooter.getInstance().setTooth(toothState);
        return false;
    }

    @Override
    public boolean execute() {
        Shooter.getInstance().setTooth(toothState);
        return true;
    }

    @Override
    public boolean updateOutputs() {
        Shooter.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() { }

}
