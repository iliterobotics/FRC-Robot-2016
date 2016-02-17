package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoReachedDefense extends AutoCommand {

    private final double MAX_DISTANCE = 5;

    private SensorInputControlSRX sensorInputControl = SensorInputControlSRX
            .getInstance();
    double rightEncoderDistance;
    double leftEncoderDistance;

    @Override
    public boolean init() {
        rightEncoderDistance = 0;
        leftEncoderDistance = 0;
        return true;
    }

    @Override
    public boolean execute() {

        DriverStation.reportError("\nCurrent getRoll(): "
                + sensorInputControl.getNavX().getRoll(), false);
        if (Math.abs(sensorInputControl.getNavX()
                .getRoll()) >= AutonomousRoutine.PITCH_CHANGE_ON_RAMP
                        + Math.abs(sensorInputControl.getInitRoll())) {
            DriverStation.reportError("\nRobot has reached the defense!",
                    false);
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // no outputs to update, only a checkpoint
        return false;
    }

    @Override
    public void reset() {
        // no outputs changed, only a checkpoint
    }

}
