package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

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
    public static final double DEF_A_ANGLE = .75, DEF_B_ANGLE = 173;
    private static final double MAX_MOTOR_SPEED_A = .5;
    private static final double MAX_MOTOR_SPEED_B = .7;
    private static final double MOTOR_SPEED_A = .2;
    private static final double MOTOR_SPEED_B = .2;
    private static final double DEGREE_MARGIN_E = 5;
    private static final double JOY_DEADZONE = .1;
    private static final double JOY_CHANGE = .005;

    private double jointASpeed; // speed that joint A will go at
    private double jointBSpeed; // speed that joint B will go at
    private double jointAAngle; // storage for updating the A angle
    private double jointBAngle; // storage for updating the B angle
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

    protected UtilityArm() {
        aP = .7;
        aI = 0.0005;
        aD = 3;
        bP = 2;
        bI = 0.00125;
        bD = 0;
        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        jointAAngle = getAngleA();
        jointBAngle = getAngleB();
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
            jointAAngle = DEF_A_ANGLE;
            jointBAngle = DEF_B_ANGLE;
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
        jointASpeed = jointAControlLoop.getPID(jointAAngle, getAngleA());
        jointBSpeed = -jointBControlLoop.getPID(jointBAngle, getAngleB());

        if (Math.abs(jointASpeed) > MAX_MOTOR_SPEED_A) {
            jointASpeed = jointASpeed > 0 ? MAX_MOTOR_SPEED_A
                    : -MAX_MOTOR_SPEED_A;
        }
        if (Math.abs(jointBSpeed) > MAX_MOTOR_SPEED_B) {
            jointBSpeed = jointBSpeed > 0 ? MAX_MOTOR_SPEED_B
                    : -MAX_MOTOR_SPEED_B;
        }

        //
        // if (jointBAngle - getAngleB() > DEGREE_MARGIN_E) {
        // jointBSpeed = -MOTOR_SPEED_B;
        // } else if (jointBAngle - getAngleB() < -DEGREE_MARGIN_E) {
        // jointBSpeed = MOTOR_SPEED_B;
        // }

        if (jointAAngle - getAngleA() < DEGREE_MARGIN_E
                && jointAAngle - getAngleA() > -DEGREE_MARGIN_E) {
            jointASpeed = 0;
        }
        if (jointBAngle - getAngleB() < DEGREE_MARGIN_E
                && jointBAngle - getAngleB() > -DEGREE_MARGIN_E) {
            jointBSpeed = 0;
        }

        DriverStation.reportError("\n\nJoint B:: Speed: " + jointBSpeed
                + " Goal: " + jointBAngle + " Current: " + getAngleB()
                + "\n P: " + jointBControlLoop.getP() + " I: "
                + jointBControlLoop.getI(1.0) + " D: "
                + jointBControlLoop.getD(), false);

        DriverStation.reportError("\n\nJoint A:: Speed: " + jointASpeed
                + " Goal: " + jointAAngle + " Current: " + getAngleA()
                + "\n P: " + jointBControlLoop.getP() + " I: "
                + jointBControlLoop.getI(1.0) + " D: "
                + jointBControlLoop.getD(), false);

        RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed,
                jointBSpeed);
    }

    /*
     * Gets angle in degrees for the bottom joint going clockwise from 0
     */
    public double getAngleA() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        double angleA = sensorInputControl.getAnalogGeneric(
                SensorType.JOINT_A_POTENTIOMETER) * CONVERSION_FACTOR;
        double zeroedA = angleA - sensorInputControl.getInitialPotAPostition();
        return zeroedA;
    }

    /*
     * Gets angle in degrees for the top joint going clockwise from 0
     */
    public double getAngleB() {
        double angleA = getAngleA();
        double angleB = angleA + 360
                - (sensorInputControl.getAnalogGeneric(
                        SensorType.JOINT_B_POTENTIOMETER) * CONVERSION_FACTOR
                - sensorInputControl.getInitialPotBPostition() + 190);

        return angleB;
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

        jointAAngle = Math.toDegrees(Math.atan2(finaly, finalx));

        double transformedX = (x - finalx);
        double transformedY = (y - finaly);
        jointBAngle = Math.toDegrees(Math.atan2(transformedY, transformedX));
        if (jointBAngle < 0) {
            jointBAngle += 360;
        }

        if (jointAAngle < DEF_A_ANGLE) {
            jointAAngle = DEF_A_ANGLE;
        }
        if (jointAAngle > 180) {
            jointAAngle = 180;
        }
        if (jointBAngle > (160 + jointAAngle)) {
            jointBAngle = (160 + jointAAngle);
        }
        if (jointBAngle < jointAAngle) {
            jointBAngle = jointAAngle;
        }

        // DriverStation.reportError("\nJointAAngle:" + jointAAngle, false);
        // DriverStation.reportError("\nJointBAngle:" + jointBAngle, false);

        jointAControlLoop.setScalingValue(jointAAngle);
        jointBControlLoop.setScalingValue(jointBAngle);

        goingToX = x;
        goingToY = y;

    }
    public boolean isFinished() {
        boolean isJointAFinished = jointAAngle - getAngleA() < DEGREE_MARGIN_E
                && jointAAngle - getAngleA() > -DEGREE_MARGIN_E;
        boolean isJointBFinished = jointBAngle - getAngleB() < DEGREE_MARGIN_E
                && jointBAngle - getAngleB() > -DEGREE_MARGIN_E;

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
            jointAControlLoop.reset();
            jointBControlLoop.reset();
            jointASpeed = 0;
            jointBSpeed = 0;
            RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed,
                    jointBSpeed);
        }
        return isJointAFinished && isJointBFinished;
    }
    public void resetPos() {
        jointAAngle = DEF_A_ANGLE;
        jointBAngle = DEF_B_ANGLE;
    }

}
