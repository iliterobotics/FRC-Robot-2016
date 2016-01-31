package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.modules.Module;

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
//        if (SensorInputControl.getInstance().isActive(
//                SensorType.TOUCH_SENSOR_TOTE_LEFT)) {
//            left = Value.kOn;
//        } else {
//            left = Value.kOff;
//        }
//        if (SensorInputControl.getInstance().isActive(
//                SensorType.TOUCH_SENSOR_TOTE_RIGHT)) {
//            right = Value.kOn;
//        } else {
//            right = Value.kOff;
//        }
    }

    @Override
    public void updateOutputs() {
    }

}
