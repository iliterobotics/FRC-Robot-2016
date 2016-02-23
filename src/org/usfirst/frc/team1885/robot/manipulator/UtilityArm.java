package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import com.ni.vision.NIVision.ConcentricRakeDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

/**
 * @author ILITE Robotics
 * @version 2/12/2016
 * 
 *          UtilityArm is the code for the side arm of the 2016 Stronghold Robot
 *          The arm features a double jointed system with a potentiometer and
 *          motor attached to each joint
 */
public class UtilityArm implements Module {

    private static UtilityArm instance;

    public static final double LENGTH_A = 17.5;
    public static final double LENGTH_B = 18;
    public static final double CONVERSION_FACTOR = 360.0 / 1024;
    public static final double DEF_A_ANGLE = 10 / CONVERSION_FACTOR,
            DEF_B_ANGLE = 173 / CONVERSION_FACTOR;
    private static final double MAX_MOTOR_SPEED_A = .5;
    private static final double MAX_MOTOR_SPEED_B = .7;
    private static final double MOTOR_SPEED_A = .2;
    private static final double MOTOR_SPEED_B = .2;
    private static final double DEGREE_MARGIN_E = 2 / CONVERSION_FACTOR;
    private static final double JOY_DEADZONE = .1;
    private static final double JOY_CHANGE = .005;

    private double jointASpeed; // speed that joint A will go at
    private double jointBSpeed; // speed that joint B will go at
    private double jointAPosition; // storage for updating the A angle
    private double jointBPosition; // storage for updating the B angle
    private double xDirection; // what direction the system needs to move in
    private double yDirection; // what direction the system needs to move in
    private double aP, aI, aD; // values for the PID to move joint A
    private double bP, bI, bD; // values for the PID to move joint B

    private double goingToX;
    private double goingToY;

    private PID jointAControlLoop; // PID to control movement of joint A
    private PID jointBControlLoop; // PID to control movement of joint B

    @SuppressWarnings("unused")
    private MotorState jointAState;
    @SuppressWarnings("unused")
    private MotorState jointBState;
    private SensorInputControlSRX sensorInputControl;
    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;

    protected UtilityArm() {
        aP = 1;
        aI = 0;
        aD = 0;
        bP = 3;
        bI = 0;
        bD = 0;

        robotControl = RobotControlWithSRX.getInstance();
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A)
                .changeControlMode(TalonControlMode.Position);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B)
                .changeControlMode(TalonControlMode.Position);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A)
                .setFeedbackDevice(FeedbackDevice.AnalogPot);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B)
                .setFeedbackDevice(FeedbackDevice.AnalogPot);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A).setPID(aP, aI,
                aD);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B).setPID(bP, bI,
                bD);

        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        jointAPosition = 0;
        jointBPosition = 0;
        xDirection = 0;
        yDirection = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        driverInputControl = DriverInputControlSRX.getInstance();
        jointAControlLoop = new PID(aP, aI, aD);
        jointBControlLoop = new PID(bP, bI, bD);
    }

    // TODO singletons cause memory leaks
    public static UtilityArm getInstance() {
        if (instance == null) {
            instance = new UtilityArm();
        }
        return instance;
    }

    @Override
    public void update() {
        // changeValues();
        if (driverInputControl.isResetButtonDown()) {
            jointAPosition = DEF_A_ANGLE;
            jointBPosition = DEF_B_ANGLE;
        }
        if (driverInputControl.isButtonDown(RobotButtonType.INCREMENT_ARM_UP)) {
            goTo(-20, 10);
        }
        // DriverStation.reportError("desired A angle:" + jointAAngle + "\n",
        // false);
        // DriverStation.reportError("desired B angle:" + jointBAngle + "\n",
        // false);
        // DriverStation.reportError("current A angle:" + getAngleA() + "\n",
        // false);
        // DriverStation.reportError("current B angle:" + getAngleB() + "\n",
        // false);
        updateOutputs();
        // DriverStation.reportError("\n\nX Distance = " + getDistanceX(),
        // false);
        // DriverStation.reportError("\nY Distance = " + getDistanceY(), false);
    }

    @Override
    public void updateOutputs() {
        // jointASpeed = jointAControlLoop.getPID(jointAAngle, getAngleA());
        // jointBSpeed = -jointBControlLoop.getPID(jointBAngle, getAngleB());

        // if (Math.abs(jointASpeed) > MAX_MOTOR_SPEED_A) {
        // jointASpeed = jointASpeed > 0 ? MAX_MOTOR_SPEED_A
        // : -MAX_MOTOR_SPEED_A;
        // }
        // if (Math.abs(jointBSpeed) > MAX_MOTOR_SPEED_B) {
        // jointBSpeed = jointBSpeed > 0 ? MAX_MOTOR_SPEED_B
        // : -MAX_MOTOR_SPEED_B;
        // }

        // if (jointBAngle - getAngleB() > DEGREE_MARGIN_E) {
        // jointBSpeed = -MOTOR_SPEED_B;
        // } else if (jointBAngle - getAngleB() < -DEGREE_MARGIN_E) {
        // jointBSpeed = MOTOR_SPEED_B;
        // }

        // if (jointAPosition - getAngleA() < DEGREE_MARGIN_E
        // && jointAPosition - getAngleA() > -DEGREE_MARGIN_E) {
        // jointASpeed = 0;
        // }
        // if (jointBPosition - getAngleB() < DEGREE_MARGIN_E
        // && jointBPosition - getAngleB() > -DEGREE_MARGIN_E) {
        // jointBSpeed = 0;
        // }

    }

    /*
     * Gets angle in degrees for the bottom joint going clockwise from 0
     */
    // public double getAngleA() {
    // sensorInputControl = SensorInputControlSRX.getInstance();
    // double angleA = sensorInputControl.getAnalogGeneric(
    // SensorType.JOINT_A_POTENTIOMETER) * CONVERSION_FACTOR;
    // double zeroedA = angleA - sensorInputControl.getInitialPotAPostition();
    // return zeroedA;
    // }

    /*
     * Gets angle in degrees for the top joint going clockwise from 0
     */
    // public double getAngleB() {
    // double angleA = getAngleA();
    // double angleB = angleA + 360
    // - (sensorInputControl.getAnalogGeneric(
    // SensorType.JOINT_B_POTENTIOMETER) * CONVERSION_FACTOR
    // - sensorInputControl.getInitialPotBPostition() + 190);
    //
    // return angleB;
    // }

    public void moveToPotValues(double potA, double potB) {
        RobotControlWithSRX.getInstance().updateArmMotors(potA, potB);
    }

    public void moveToDegreeVlaue(double degreeA, double degreeB) {
        RobotControlWithSRX.getInstance().updateArmMotors(
                degreeA / CONVERSION_FACTOR + SensorInputControlSRX
                        .getInstance().INITIAL_POT_A_POSITION,
                degreeB / CONVERSION_FACTOR + SensorInputControlSRX
                        .getInstance().INITIAL_POT_B_POSITION);
    }

    /**
     * A simple down to earth equation for calculating the angles required to
     * acieve a point
     * 
     * @param x
     *            the x coordinate of the new end-point
     * @param y
     *            the y coordinate of the new end-point
     */
    public void goTo(double x, double y) {

        if (x > LENGTH_A - 2 && y < LENGTH_B - 2) {
            x = LENGTH_A - 2;
            y = LENGTH_B - 2;
        }

        double p = Math.sqrt((x * x) + (y * y));
        double k = ((p * p) + LENGTH_A * LENGTH_A - LENGTH_B * LENGTH_B)
                / (2 * p);

        double x1 = (x * k) / p
                + (y / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);
        double y1 = (y * k) / p
                - (x / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);

        double x2 = (x * k) / p
                - (y / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);
        double y2 = (y * k) / p
                + (x / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);

        double finaly = 0;
        double finalx = 0;
        if (y1 < 0) {
            finaly = y2;
            finalx = x2;
        } else {
            finaly = y1;
            finalx = x1;
        }

        // DriverStation.reportError("\nfinalX:" + finalx, false);
        // DriverStation.reportError("\nfinalY:" + finaly, false);
        //
        // DriverStation.reportError("\ntanA:" + finalx / finaly, false);
        // DriverStation.reportError("\ntanB:" + (y - finaly) / (x - finalx),
        // false);

        jointAPosition = Math.toDegrees(Math.atan2(finaly, finalx));

        double transformedX = (x - finalx);
        double transformedY = (y - finaly);
        jointBPosition = Math.toDegrees(Math.atan2(transformedY, transformedX));
        DriverStation.reportError("\nJoint A Position before conversion 1: "
                + (jointAPosition + SensorInputControlSRX
                        .getInstance().INITIAL_POT_A_POSITION
                        * CONVERSION_FACTOR),
                false);
        DriverStation.reportError("\nJoint B Position before conversion 1: "
                + (jointBPosition + SensorInputControlSRX
                        .getInstance().INITIAL_POT_B_POSITION
                        * CONVERSION_FACTOR),
                false);

        if (jointBPosition < 0) {
            jointBPosition += 360;
        }

        if (jointAPosition < DEF_A_ANGLE) {
            jointAPosition = DEF_A_ANGLE;
        }
        if (jointAPosition > 180) {
            jointAPosition = 180;
        }
        if (jointBPosition > (160 + jointAPosition)) {
            jointBPosition = (160 + jointAPosition);
        }
        if (jointBPosition < jointAPosition) {
            jointBPosition = jointAPosition;
        }

        // DriverStation.reportError("\nJointAAngle:" + jointAAngle, false);
        // DriverStation.reportError("\nJointBAngle:" + jointBAngle, false);

        // jointAControlLoop.setScalingValue(jointAAngle);
        // jointBControlLoop.setScalingValue(jointBAngle);

        DriverStation.reportError("\nJoint A Position before conversion 2: "
                + (jointAPosition + SensorInputControlSRX
                        .getInstance().INITIAL_POT_A_POSITION
                        * CONVERSION_FACTOR),
                false);
        DriverStation.reportError("\nJoint B Position before conversion 2: "
                + (jointBPosition + SensorInputControlSRX
                        .getInstance().INITIAL_POT_B_POSITION
                        * CONVERSION_FACTOR),
                false);

        jointAPosition = jointAPosition / CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_A_POSITION;
        jointBPosition = jointBPosition / CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_B_POSITION;

        DriverStation.reportError("\nJoint A Position: " + jointAPosition,
                false);
        DriverStation.reportError("\nJoint B Position: " + jointBPosition,
                false);

        RobotControlWithSRX.getInstance().updateArmMotors(jointAPosition,
                jointBPosition);

        goingToX = x;
        goingToY = y;

    }
    public boolean isFinished() {
        boolean isJointAFinished = robotControl.getTalons()
                .get(RobotMotorType.ARM_JOINT_A).get()
                - jointAPosition < DEGREE_MARGIN_E
                && robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A)
                        .get() - jointAPosition > -DEGREE_MARGIN_E;
        boolean isJointBFinished = robotControl.getTalons()
                .get(RobotMotorType.ARM_JOINT_B).get()
                - jointBPosition < DEGREE_MARGIN_E
                && robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B)
                        .get() - jointBPosition > -DEGREE_MARGIN_E;

        // DriverStation.reportError("\nAre finished? ("
        // + (isJointAFinished ? "true" : "false") + ", "
        // + (isJointBFinished ? "true" : "false") + ")\n Joint Angle A: "
        // + jointAAngle + " -- Joint Angle B: " + jointBAngle
        // + "\n Get Angle A: " + getAngleA() + " -- Get Angle B: "
        // + getAngleB() + "\nInitial B value: "
        // + sensorInputControl.INITIAL_POT_B_POSITION +
        // "\nRaw angle B:" + sensorInputControl.getAnalogGeneric(
        // SensorType.JOINT_B_POTENTIOMETER) * CONVERSION_FACTOR +
        // "\nRaw pot val:" + sensorInputControl.getAnalogGeneric(
        // SensorType.JOINT_B_POTENTIOMETER), false);
        if (isJointAFinished && isJointBFinished) {
            // jointAControlLoop.reset();
            // jointBControlLoop.reset();
            // jointASpeed = 0;
            // jointBSpeed = 0;
            // RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed,
            // jointBSpeed);
        }
        return isJointAFinished && isJointBFinished;
    }
    public void resetPos() {
        jointAPosition = DEF_A_ANGLE;
        jointBPosition = DEF_B_ANGLE;
    }

}
