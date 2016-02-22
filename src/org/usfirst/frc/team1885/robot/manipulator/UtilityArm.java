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

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
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

    private double bleh;

    public static final double LENGTH_A = 17.5;
    public static final double LENGTH_B = 18;
    public static final double CONVERSION_FACTOR = 360.0 / 1024;
    public static final double DEF_A_ANGLE = .5, DEF_B_ANGLE = 173;
    private static final double MAX_MOTOR_SPEED_A = .5;
    private static final double MAX_MOTOR_SPEED_B = .7;
    private static final double MOTOR_SPEED_A = .2;
    private static final double MOTOR_SPEED_B = .2;
    private static final double DEGREE_MARGIN_E = 5;
    private static final double JOY_DEADZONE = .1;
    private static final double JOY_CHANGE = .005;

    private double jointAPos; // speed that joint A will go at
    private double jointBPos; // speed that joint B will go at
    private double jointAAngle; // storage for updating the A angle
    private double jointBAngle; // storage for updating the B angle
    private double xDirection; // what direction the system needs to move in
    private double yDirection; // what direction the system needs to move in
    private double aP, aI, aD; // values for the PID to move joint A
    private double bP, bI, bD; // values for the PID to move joint B

    private double goingToX;
    private double goingToY;

    @SuppressWarnings("unused")
    private MotorState jointAState;
    @SuppressWarnings("unused")
    private MotorState jointBState;
    private SensorInputControlSRX sensorInputControl;
    private DriverInputControlSRX driverInputControl;

    protected UtilityArm() {

        aP = 10;
        aI = 0;
        aD = 0;
        // bP = 1;
        // bI = 0;
        // bD = 0;
        bP = 10;
        bI = 0;
        bD = 0;
        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        xDirection = 0;
        yDirection = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        driverInputControl = DriverInputControlSRX.getInstance();

        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_A)
                .changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_A)
                .setFeedbackDevice(FeedbackDevice.AnalogPot);
        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_A).setPID(aP, aI, aD);

        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_B)
                .changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_B)
                .setFeedbackDevice(FeedbackDevice.AnalogPot);
        RobotControlWithSRX.getInstance().getTalons()
                .get(RobotMotorType.ARM_JOINT_B).setPID(bP, bI, bD);
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
        // if
        // (driverInputControl.isButtonDown(RobotButtonType.INCREMENT_ARM_UP)) {
        // goTo(-20, 10);
        // }
        DriverStation.reportError("\ndesired A Angle:" + jointAAngle
                + " current Angle A:" + getAngleA() + "\n", false);
        DriverStation.reportError(
                "\nGet PID: " + RobotControlWithSRX.getInstance().getTalons()
                        .get(RobotMotorType.ARM_JOINT_A).pidGet(),
                false);
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

        // if (Math.abs(jointAPos) > MAX_MOTOR_SPEED_A) {
        // jointAPos = jointAPos > 0 ? MAX_MOTOR_SPEED_A
        // : -MAX_MOTOR_SPEED_A;
        // }
        // if (Math.abs(jointBPos) > MAX_MOTOR_SPEED_B) {
        // jointBPos = jointBPos > 0 ? MAX_MOTOR_SPEED_B
        // : -MAX_MOTOR_SPEED_B;
        // }

        //
        // if (jointBAngle - getAngleB() > DEGREE_MARGIN_E) {
        // jointBSpeed = -MOTOR_SPEED_B;
        // } else if (jointBAngle - getAngleB() < -DEGREE_MARGIN_E) {
        // jointBSpeed = MOTOR_SPEED_B;
        // }

        // if (jointAAngle - getAngleA() < DEGREE_MARGIN_E
        // && jointAAngle - getAngleA() > -DEGREE_MARGIN_E) {
        // jointAPos = 0;
        // }
        // if (jointBAngle - getAngleB() < DEGREE_MARGIN_E
        // && jointBAngle - getAngleB() > -DEGREE_MARGIN_E) {
        // jointBPos = 0;
        // }

        jointAPos = jointAAngle + sensorInputControl.getInitialPotAPostition();
        jointBPos = jointBAngle + sensorInputControl.getInitialPotBPostition();

        DriverStation.reportError("\njoinAAngle: " + jointAAngle, false);

        RobotControlWithSRX.getInstance().updateArmMotors(jointAPos, jointBPos);
    }

    /*
     * Gets angle in degrees for the bottom joint going clockwise from 0
     */
    public double getAngleA() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        double angleA = sensorInputControl.getAnalogGeneric(
                SensorType.JOINT_A_POTENTIOMETER) / CONVERSION_FACTOR;
        double zeroedA = angleA - sensorInputControl.getInitialPotAPostition();
        // DriverStation.reportError(
        // "\ninitialPotAngle: "
        // + sensorInputControl.getInitialPotAPostition()
        // + "\ngetAngleA returns: " + zeroedA
        // + "\nAnalog Generic: "
        // + sensorInputControl.getAnalogGeneric(
        // SensorType.JOINT_A_POTENTIOMETER)
        // + "\n",
        // false);
        return zeroedA;
    }

    /*
     * Gets angle in degrees for the top joint going clockwise from 0
     */
    public double getAngleB() {
        double angleA = getAngleA();
        double angleB = angleA + 360
                - (sensorInputControl.getAnalogGeneric(
                        SensorType.JOINT_B_POTENTIOMETER) / CONVERSION_FACTOR
                - sensorInputControl.getInitialPotBPostition() + 190);
        return angleB;
    }

    /*
     * Finds the conversion from the joystick twist value to y movement in
     * inches
     */

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

        goingToX = x;
        goingToY = y;

    }

    /*
     * Takes the change in x and y movement and converts them to angular change
     */
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
        // if (isJointAFinished && isJointBFinished) {
        //// jointASpeed = 0;
        //// jointBSpeed = 0;
        // RobotControlWithSRX.getInstance().updateArmMotors(jointAPos,
        // jointBPos);
        // }
        return isJointAFinished && isJointBFinished;
    }
    public void resetPos() {
        jointAAngle = DEF_A_ANGLE;
        jointBAngle = DEF_B_ANGLE;
    }

}
