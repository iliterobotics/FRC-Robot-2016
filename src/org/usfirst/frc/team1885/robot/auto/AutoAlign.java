package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * This class rotates the robot into it's initial facing - yaw - position.
 * 'Initial facing' position being the position at which the robot was when this
 * class was created. It rotates to that position by anchoring one side of the
 * drive train and negatively powering the other.
 * 
 * @author ILITE Robotics
 * @version 2/13/2016
 */
public class AutoAlign extends AutoCommand {

    private final double P, I, D;
    private final double ALIGNMENT_ERROR = .5;
    private final double MIN_SPEED = 0.225;
    private double targetDegree;
//    private PID pid;
    private SensorInputControlSRX sensorInputControl;
    private double rightDrivePower;
    private double leftDrivePower;
    private double initialYaw;
    private static final double RADIUS = 16.5;
    
    public AutoAlign() {
        this(0);
    }

    public AutoAlign(double degree) {
        sensorInputControl = SensorInputControlSRX.getInstance();
        initialYaw = (sensorInputControl.getYaw() + 360) % 360;
        targetDegree = (degree - initialYaw + 360) % 360;
//        double scale = 180 / Math.abs(degree == 0 ? 1 : degree);
//        P = 1 / scale;
//        I = 0.001 / scale;
//        D = 5 / scale;
//        pid = new PID(P, I, D);
//        pid.setScalingValue(targetDegree);
        
        P = 2.0;
        I = 0.0002;
        D = 0;
    }

    @Override
    public boolean init() {
        rightDrivePower = leftDrivePower = 0;
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).changeControlMode(TalonControlMode.Position);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(P, I, D);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(P, I, D);
        
        double currentTicksLeft = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get();
        double currentTicksRight =RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).set(Math.toRadians(targetDegree) * RADIUS /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksLeft);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).set(Math.toRadians(targetDegree) * RADIUS /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksRight);
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = sensorInputControl.getYaw();

//        if (yaw < 0) {
//            yaw += 360;
//        }

        double difference = (yaw - targetDegree);

//        rightDrivePower = pid.getPID(difference < -180 ? -360 : 0, difference);
//        leftDrivePower = rightDrivePower * -1;

        DriverStation.reportError("\n Degree to turn : " + targetDegree
                + " --- Normalized yaw: " + yaw + "\n Pid Speed:: "
                + rightDrivePower + "\n difference:: " + difference, false);

//        if (leftDrivePower > 0) {
//            leftDrivePower = (leftDrivePower < MIN_SPEED ? MIN_SPEED
//                    : leftDrivePower);
//        } else if (leftDrivePower < 0) {
//            leftDrivePower = (leftDrivePower > -MIN_SPEED ? -MIN_SPEED
//                    : leftDrivePower);
//        }
//
//        if (rightDrivePower > 0) {
//            rightDrivePower = (rightDrivePower < MIN_SPEED ? MIN_SPEED
//                    : rightDrivePower);
//        } else if (rightDrivePower < 0) {
//            rightDrivePower = (rightDrivePower > -MIN_SPEED ? -MIN_SPEED
//                    : rightDrivePower);
//        }
//
//        if (Math.abs((difference < 0 ? difference + 360 : difference)) < ALIGNMENT_ERROR) {
//            DriverStation.reportError("\nAligned.", false);
//            this.reset();
//            return true;
//        }
//        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDrivePower);
//        DrivetrainControl.getInstance().setRightDriveSpeed(rightDrivePower);
        
        return false;
    }

    @Override
    public boolean updateOutputs() {
//        RobotControlWithSRX.getInstance().updateDriveSpeed(leftDrivePower,
//                rightDrivePower);
        return false;
    }

    @Override
    public void reset() {
//        pid.reset();
//        leftDrivePower = 0;
//        rightDrivePower = 0;
//        DrivetrainControl.getInstance().setLeftDriveSpeed(0);
//        DrivetrainControl.getInstance().setRightDriveSpeed(0);
    }
}