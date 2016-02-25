package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * Waits until the robot has traversed a certain distance. Moving forward 1 in
 * then backwards 1 equates to traveling 2 in. This is determined by the left
 * wheel's encoder values and the circumference of the wheels. Generic encoder
 * to distance math is used.
 * 
 * @author ILITE Robotics
 * @version <2/13/2016>
 */
public class AutoDriveDistance extends AutoCommand {
    private SensorInputControlSRX sensorInputControl;

    private double distance; // Inputed distance to traverse
    private double initDisLeft; // Initial distance of left drive train side
    private double initDisRight; // Initial distance of right drive train side
    private double disLeft; // Current distance of left drive train side
    private double disRight; // Current distance of right drive train side

    private boolean doesStop; // If the drive train should stop after traversing
    private boolean isLeftFinished; // If the left drive train side is finished
                                    // traversing
    private boolean isRightFinished; // If the right drive train side is
                                     // finished traversing
    private double differenceLeft, differenceRight;
    
    private final double ERROR;
    private double P;

    /**
     * @param distance
     *            Distance to travel in inches
     * @param doesStop
     *            If it should stop at the end of the distance
     */
    public AutoDriveDistance(double distance, boolean doesStop) {
        ERROR = 4;
        differenceLeft = differenceRight = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        this.distance = distance;
        this.doesStop = doesStop;
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Position);
    }
    
    public AutoDriveDistance(double distance, boolean doesStop, double P){
        this(distance, doesStop);
        this.P = P;
    }

    @Override
    public boolean execute() {
        disLeft = sensorInputControl.getEncoderDistance(SensorType.LEFT_ENCODER)
                - initDisLeft;
        disRight = sensorInputControl
                .getEncoderDistance(SensorType.RIGHT_ENCODER) - initDisRight;

        // DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        // DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);

        differenceLeft = disLeft - distance;
        differenceRight = disRight - distance;

        isLeftFinished = Math.abs(differenceLeft) < ERROR;
        isRightFinished = Math.abs(differenceRight) < ERROR;

//        DriverStation.reportError("\n\nDistance Left:: " + disLeft
//                + "\ndistance Right:: " + disRight + "\nDifference Left:: "
//                + differenceLeft + "\nDifference Right:: " + differenceRight,
//                false);

//        DriverStation.reportError("\nRight Drive Speed:: " + rightDriveSpeed
//                + "\nLeft Drive Speed:: " + leftDriveSpeed, false);

        // DriverStation.reportError(
        // "\nDisRight: " + disRight + ", initDisRight: " + initDisRight,
        // false);
        // DriverStation.reportError(
        // "\ndisLeft: " + disLeft + ", initDisLeft: " + initDisLeft,
        // false);

        if (!doesStop && isRightFinished && isLeftFinished) {
//            DriverStation.reportError("\nFinished traveling distance!"
//                    + System.currentTimeMillis(), false);
            return true;
        } else if (isRightFinished && isLeftFinished) {
//            DriverStation.reportError(
//                    "\nFinished traveling distance! Stopping.", false);
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        //no outputs to update, controlled by SRX PID
        return false;
    }

    @Override
    public boolean init() {
        CANTalon left = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE);
        CANTalon right = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE);
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(P, left.getI(), left.getD());
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(P, right.getI(), right.getD());
        
        double currentTicksLeft = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get();
        double currentTicksRight =RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).set(distance /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksLeft);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).set(distance /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksRight);
        return true;
    }

    @Override
    public void reset() {
        // No values to reset
    }

}