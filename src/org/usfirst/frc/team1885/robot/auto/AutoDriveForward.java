package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.TruePID;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import com.sun.org.glassfish.external.statistics.annotations.Reset;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class AutoDriveForward {

    private static final double ALLOWABLE_MOTOR_DIFF = 0.05; // in percentage

    private TruePID leftPID;
    private TruePID rightPID;

    private int initialLeftTicks;
    private int initialRightTicks;

    private static final double P = 0.4;
    private static final double I = 0.1;
    private static final double D = 0.01;
    private static final int FULL_ERROR = 500;

    private final SensorType LEFT_ENCODER;
    private final SensorType RIGHT_ENCODER;

    private RobotControlWithSRX srx;

    /** generic constructor */
    public AutoDriveForward(SensorType leftEncoder, SensorType rightEncoder) {
        LEFT_ENCODER = leftEncoder;
        RIGHT_ENCODER = rightEncoder;

        srx = RobotControlWithSRX.getInstance();
        
        leftPID = new TruePID(P, I, D, FULL_ERROR);
        rightPID = new TruePID(P, I, D, FULL_ERROR);

        leftPID.reset();
        rightPID.reset();

        initialLeftTicks = srx.getSensor().get(LEFT_ENCODER).getEncPosition();
        initialRightTicks = srx.getSensor().get(RIGHT_ENCODER).getEncPosition();
    }

    public void driveForward(double leftMotor, double rightMotor) {
        
        DriverStation.reportError("\n\nIleftTicks:" + initialLeftTicks, false);
        DriverStation.reportError("\nIrightTicks:" + initialRightTicks, false);
        
        int leftTicks = srx.getSensor().get(LEFT_ENCODER).getEncPosition() - initialLeftTicks;
        int rightTicks = srx.getSensor().get(RIGHT_ENCODER).getEncPosition() - initialRightTicks;

        DriverStation.reportError("\nleftTicks:" + leftTicks, false);
        DriverStation.reportError("\nrightTicks:" + rightTicks, false);
        
        if (Math.abs(leftTicks) > Math.abs(rightTicks)) {
            double pidAdjustment = rightPID.getPID(leftTicks, rightTicks);
            rightMotor += pidAdjustment;
            DriverStation.reportError("\nradjustment:" + pidAdjustment, false);
        } else if (Math.abs(rightTicks) > Math.abs(leftTicks)) {
            double pidAdjustment = leftPID.getPID(leftTicks, rightTicks);
            leftMotor += pidAdjustment;
            DriverStation.reportError("\nladjustment:" + pidAdjustment, false);
        }
        
        DriverStation.reportError("\nleftMotor:" + leftMotor, false);
        DriverStation.reportError("\nrightMotor:" + rightMotor, false);
        
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftMotor);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightMotor);
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftMotor, rightMotor);
    }
}
