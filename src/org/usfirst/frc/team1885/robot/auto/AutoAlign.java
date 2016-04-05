package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;

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

    private final double ALIGNMENT_ERROR = 50; //TICKS
    private double targetDegree, absoluteTarget;
    private double direction;
    private double tickGoalRight, tickGoalLeft;
    private SensorInputControlSRX sensorInputControl;
    public static final double TURN_RADIUS = 16;
    
    public AutoAlign() {
        this(0);
    }

    /**
     * Aligns to degree degree relative to initial yaw
     * @param degree value to align to
     */
    public AutoAlign(double degree) {
        sensorInputControl = SensorInputControlSRX.getInstance();
        targetDegree = degree;
        absoluteTarget = (degree + 360) % 360;
    }

    @Override
    public boolean init() {
        DrivetrainControl.getInstance().setControlMode(TalonControlMode.Position);
        
        double currentTicksLeft = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get();
        double currentTicksRight = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
        
        double initialYaw = SensorInputControlSRX.getInstance().getYaw();
        
        direction = (targetDegree - initialYaw) < 0 ? -1 : 1;
//        DriverStation.reportError("\nPure Target:: " + targetDegree, false);
        targetDegree = Math.abs(targetDegree - initialYaw);
        if(targetDegree > 180){
            targetDegree -= 360;
        }
        if(targetDegree < -180){
            targetDegree += 360;
        }
//        DriverStation.reportError("\n Direction:: " + direction + "  targetDegree:: " + targetDegree, false);
        
        tickGoalLeft = direction * (Math.toRadians(targetDegree) * TURN_RADIUS) /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksLeft;
        tickGoalRight = direction * (Math.toRadians(targetDegree) * TURN_RADIUS) /(Math.PI * RobotConfiguration.WHEEL_DIAMETER) * DrivetrainControl.TICKS_IN_ROTATION + currentTicksRight;
        
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).set(tickGoalLeft);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).set(tickGoalRight);
        return true;
    }

    @Override
    public boolean execute() {
        double yaw = (sensorInputControl.getYaw() + 360) % 360;
        double difference = (yaw - absoluteTarget);

//        DriverStation.reportError("\n Degree to turn : " + targetDegree
//                + " --- Normalized yaw: " + yaw + "\n difference:: " + difference, false);
        
        double differenceLeft = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.LEFT_DRIVE).get() - tickGoalLeft;
        double differenceRight = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.RIGHT_DRIVE).get() - tickGoalRight;
        
        if (Math.abs(differenceLeft) < ALIGNMENT_ERROR && Math.abs(differenceRight) < ALIGNMENT_ERROR) {
            DriverStation.reportError("\nAligned.", false);
            this.reset();
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        //no outputs, controlled by SRX PID
        return false;
    }

    @Override
    public void reset() {
    }
}