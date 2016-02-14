package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

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
        /*
         * Is this suppose to be getRoll()?
         */

        if (sensorInputControl.getNavX()
                .getRoll() >= AutonomousRoutine.PITCH_CHANGE_ON_RAMP) {
            return true;
        }
        leftEncoderDistance = sensorInputControl
                .getEncoderPos(SensorType.LEFT_ENCODER);
        rightEncoderDistance = sensorInputControl
                .getEncoderPos(SensorType.RIGHT_ENCODER);
        if (leftEncoderDistance >= MAX_DISTANCE
                && rightEncoderDistance >= MAX_DISTANCE) {
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
