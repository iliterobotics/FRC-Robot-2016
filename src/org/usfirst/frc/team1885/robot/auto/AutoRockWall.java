package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class AutoRockWall extends AutoCommand {
    private final double WAIT_TIME = 500; // half a second
    private Stage stage;
    private AutoCrossedDefense crossedDefense;
    private AutoReachedDefense tiltDown;
    private AutoWait pause;
    private double leftDriveSpeed;
    private double rightDriveSpeed;

    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Speed);
        reset();
        crossedDefense = new AutoCrossedDefense();
        tiltDown = new AutoReachedDefense();
        pause = new AutoWait(WAIT_TIME);
        stage = Stage.FIRST;
        return true;
    }

    public boolean execute() {
        double roll = SensorInputControlSRX.getInstance().getRoll();
        // DriverStation.reportError("\nRight side: " + leftDriveSpeed
        // + " --- Left side: " + rightDriveSpeed, false);
        switch (stage) {
        case FIRST:
            if (crossedDefense.execute()) {
                stage = Stage.SECOND;
            }
            break;
        case SECOND:
            if (tiltDown.execute()) {
                stage = Stage.THIRD;
                crossedDefense = new AutoCrossedDefense();
            }
            break;
        case THIRD:
            if (pause.execute()) {
                stage = Stage.FINISHED;
            }
            break;
        default:
            return crossedDefense.execute();
        }

        if (roll > 4.5) {
            leftDriveSpeed = 0.5 + (roll / 50) * (.5);
            rightDriveSpeed = 0.5 + (roll / 50) * (.5);
        } else if (roll < -4.5) {
            leftDriveSpeed = 0.5 - (-roll / 50) * (.5);
            rightDriveSpeed = 0.5 - (-roll / 50) * (.5);
        }
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        DrivetrainControl.getInstance().updateOutputs();
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

    private enum Stage {
        FIRST/* Initial defense reach */, SECOND/* Move until flat on defense */, THIRD/*
                                                                                        * move
                                                                                        * until
                                                                                        * flat
                                                                                        * on
                                                                                        * ground
                                                                                        */, FINISHED
    }

}
