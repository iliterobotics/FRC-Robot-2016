package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * 
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoReachedDefense extends AutoCommand {

    private SensorInputControlSRX sensorInputControl = SensorInputControlSRX
            .getInstance();

    @Override
    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Speed);
        return true;
    }

    @Override
    public boolean execute() {
        if (Math.abs(sensorInputControl.getNavX().getRoll()) >= Math.abs(AutonomousRoutine.PITCH_CHANGE_ON_RAMP + sensorInputControl.getInitRoll())) {
//            DriverStation.reportError("\nReached Defense", false);
            return true;
        }
        if(timeOut()){
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // no outputs changed, only a checkpoint
    }

}
