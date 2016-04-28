package org.usfirst.frc.team1885.robot.modules.drivetrain;

import org.usfirst.frc.team1885.robot.common.type.DriveMode;
import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class DrivetrainControl implements Module {
    /**
     * drive mode where you can only move straight using the right joystick
     */
    public static double TICKS_IN_ROTATION = 1024;
    private double leftDriveSpeed;
    private double rightDriveSpeed;
    private DriveMode driveMode;
    private GearState gearState;
    /** in rps */
    private double maxSpeed;
    private final double diameter;
    private final double circumference;
    private DriverInputControlSRX driverInput;
    private RobotControlWithSRX robotSRX;
    private boolean isTurning;
    public static final double NUDGE_POWER = 0.15;
    public static final double NUDGE_POWER_TURN = 0.75;
    private static DrivetrainControl instance;
    private boolean gear;

    private static final double speedP = .25;
    private static final double speedI = 0.0;
    private static final double speedD = 0.0;
    
    private static final double positionP = 2;
    private static final double positionI = 0.0001;
    private static final double positionD = 0;
    
    public static final boolean HIGH_GEAR = true;
    public static final boolean LOW_GEAR = false;

    private DrivetrainControl(final double d, final double m) {
        maxSpeed = m;
        diameter = d;
        circumference = Math.PI * (diameter);
        driveMode = DriveMode.TANK;
        gear = HIGH_GEAR;
        driverInput = DriverInputControlSRX.getInstance();
        robotSRX = RobotControlWithSRX.getInstance();

        setControlMode(TalonControlMode.Speed);
    }
    public static synchronized DrivetrainControl getInstance() {
        if (instance == null) {
            instance = new DrivetrainControl(RobotConfiguration.WHEEL_DIAMETER, 10.5/*ft/s*/);
        }
        return instance;
    }
    public void init(){
        
    }
    public boolean getIsTurning() {
        return isTurning;
    }
    public void update() {
            gear = HIGH_GEAR;
        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.GEAR_SHIFT)) {
            maxSpeed = 15.0;
            gear = LOW_GEAR;
        }
//        DriverStation.reportError("\nGear: " + gear, false);
    }

    public void update(double leftJoystick, double rightJoystick) {
        if (!isTurning || (leftJoystick > 0 || rightJoystick > 0)) {
            isTurning = false;
            leftDriveSpeed = leftJoystick;
            rightDriveSpeed = rightJoystick;

            if (Math.abs(leftJoystick
                    - rightJoystick) < DriverInputControlSRX.DEADZONE) {
                leftDriveSpeed = rightDriveSpeed = (leftDriveSpeed
                        + rightDriveSpeed) / 2;
            }

            // leftDriveSpeed = DriverInputControlSRX.expScale(leftDriveSpeed);
            // rightDriveSpeed =
            // DriverInputControlSRX.expScale(rightDriveSpeed);
        } else if (isTurning) {
            leftDriveSpeed = leftJoystick;
            rightDriveSpeed = rightJoystick;
        }
    }
    /**
     * @return the leftDriveSpeed
     */
    public double getLeftDriveSpeed() {
        return leftDriveSpeed;
    }
    /**
     * @param leftDriveSpeed
     *            the leftDriveSpeed to set
     */
    public void setLeftDriveSpeed(double leftDriveSpeed) {
        this.leftDriveSpeed = leftDriveSpeed;
    }
    /**
     * @return the rightDriveSpeed
     */
    public double getRightDriveSpeed() {
        return rightDriveSpeed;
    }
    /**
     * @param rightDriveSpeed
     *            the rightDriveSpeed to set
     */
    public void setRightDriveSpeed(double rightDriveSpeed) {
        this.rightDriveSpeed = rightDriveSpeed;
        ;
    }
    /**
     * @return the driveMode
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }
    /**
     * changes driveMode from TANK drive to STRAIGHT drive STRAIGHT only uses
     * the right joystick to drive straight
     */
    public void toggleDriveMode() {
        if (driveMode == DriveMode.TANK) {
            driveMode = DriveMode.STRAIGHT;
        } else {
            driveMode = DriveMode.TANK;
        }
    }
    /**
     * @param driveSpeed
     *            sets both drive speeds to a single speed controlled by the
     *            right joystick
     */
    public void straightDrive(double driveSpeed) {
        this.rightDriveSpeed = driveSpeed;
        this.leftDriveSpeed = driveSpeed;
    }
    
    public void gearShift(boolean gear){
        this.gear = gear;
    }
    
    public void setControlMode(TalonControlMode mode){
        robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).changeControlMode(mode);
        robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).changeControlMode(mode);
        switch(mode){
        case Speed:
            robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(speedP, speedI, speedD);
            robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(speedP, speedI, speedD);
            robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).set(0);
            robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).set(0);
            break;
        case Position:
            robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).setPID(positionP, positionI, positionD);
            robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).setPID(positionP, positionI, positionD);
            break;
        default:
        }
    }

    public void updateOutputs() {
        // 100 represents conversion from seconds in the max speed to the .01
        // second rate that the talons takes
        double leftDriveVelocity = -leftDriveSpeed * maxSpeed
                / (Math.PI * RobotConfiguration.WHEEL_DIAMETER / 12.0)
                * TICKS_IN_ROTATION;
        double rightDriveVelocity = rightDriveSpeed * maxSpeed
                / (Math.PI * RobotConfiguration.WHEEL_DIAMETER / 12.0)
                * TICKS_IN_ROTATION;
            robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).set(leftDriveVelocity);
            robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).set(rightDriveVelocity);
            
            double getR = robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).get();
            double getL = robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).get();

//         DriverStation.reportError("\nGoal:: Left: " + leftDriveVelocity + " Right: " + rightDriveVelocity + "", false);
//        DriverStation.reportError(
//                        "\nSpeed:: Left: " + -leftDriveSpeed + " Right: " + rightDriveSpeed, false);
//        DriverStation.reportError("\nOutput Value:: Left: " + robotSRX.getTalons().get(RobotMotorType.LEFT_DRIVE).get() + " Right: " + robotSRX.getTalons().get(RobotMotorType.RIGHT_DRIVE).get(), false);
        RobotControlWithSRX.getInstance()
                .updateSingleSolenoid(RobotPneumaticType.GEAR_SHIFT, gear);
    }
}
