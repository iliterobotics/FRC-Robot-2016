package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoPortcullis extends AutoCommand {
    SensorInputControlSRX sensorControl;
    AutoDriveDistance halfWay; 
    AutoDriveDistance secondHalf;
    public static final double HALF_DISTANCE = -4.2 * 12 / 2;

    @Override
    public boolean init() {
        halfWay = new AutoDriveDistance(HALF_DISTANCE, .1);
        secondHalf = new AutoDriveDistance(HALF_DISTANCE, .1);
        RobotControlWithSRX.getInstance().updateIntakeMotor(ActiveIntake.INTAKE_SPEED);
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeUp);
        
        
        return false;
    }

    @Override
    public boolean execute() {
        if( halfWay.execute()){
           RobotControlWithSRX.getInstance().updateIntakeMotor(0);
           ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
           secondHalf.execute();
           return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
