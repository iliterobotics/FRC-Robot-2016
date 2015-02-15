package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.Relay.Value;

public class Alignment implements Module {

    private Value left;
    private Value right;
    private static Alignment instance;

    protected Alignment() {
        left = Value.kOff;
        right = Value.kOff;
    }

    public static Alignment getInstance() {
        if (instance == null) {
            instance = new Alignment();
        }
        return instance;
    }

    @Override
    public void update() {
        if (SensorInputControl.getInstance().isActive(
                SensorType.TOUCH_SENSOR_TOTE_LEFT)) {
            left = Value.kOn;
        } else {
            left = Value.kOff;
        }
        if (SensorInputControl.getInstance().isActive(
                SensorType.TOUCH_SENSOR_TOTE_RIGHT)) {
            right = Value.kOn;
        } else {
            right = Value.kOff;
        }
    }

    @Override
    public void updateOutputs() {
        RobotControl.getInstance().updateRelay(RobotMotorType.ALIGNMENT_LEFT,
                left);
        RobotControl.getInstance().updateRelay(RobotMotorType.ALIGNMENT_RIGHT,
                right);
    }

}
