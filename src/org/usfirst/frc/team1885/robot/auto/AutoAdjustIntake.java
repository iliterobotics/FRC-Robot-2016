package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.ActiveIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoAdjustIntake extends AutoCommand{

    @Override
    public boolean init() {
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
        return true;
    }

    @Override
    public boolean execute() {
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
        ActiveIntake.getInstance().updateOutputs();
        // TODO Auto-generated method stub
        return true;
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
