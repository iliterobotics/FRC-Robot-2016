package org.usfirst.frc.team1885.robot.manipulator;

import java.io.IOException;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import dataclient.DataServerWebClient;
import dataclient.robotdata.arm.ArmStatus;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author ILITE Robotics
 * @version 2/25/2016
 * 
 *          UtilityArm is the code for the side arm of the 2016 Stronghold Robot
 *          The arm features a double jointed system with a potentiometer and
 *          motor attached to each joint
 */
public class UtilityArm implements Module {

    private static UtilityArm instance;

    public static final double LENGTH_A = 18; // length of arm A
    public static final double LENGTH_B = 18.5; // length of arm B
    public static final double CONVERSION_FACTOR = (1024 * 4) / 360.0; // multiplier
    // to convert
    // from degrees
    // to ticks
    private static final double POSITION_MARGIN_ERR = 5 * CONVERSION_FACTOR;
    private static final double FRAME_LENGTH = 5;
    private final double RESET_A_POSITION;
    private final double RESET_B_POSITION;
    private final double BOUNDARY = 13;
    private final double X_MAX_BACK_REACH = 9;
    private final double Y_MAX_UP_REACH = 33;
    private final double Y_MAX_DOWN_REACH = -10;
    private final double DEAD_ZONE_X = .2;
    private final double DEAD_ZONE_Y = .2;
    private final double INCREMENT_RATE = 1 / 10.0; // Rate at which xCoord and
                                                    // yCoord are incremented

    private double jointAPosition; // storage for updating the A angle
    private double jointBPosition; // storage for updating the B angle
    private double jointADegree;
    private double jointBDegree;
    private double aP, aI, aD; // values for the PID to move joint A
    private double bP, bI, bD; // values for the PID to move joint B

    private double xCoord; // Current x Coordinate
    private double yCoord; // Current y Coordinate
    private double xModifier; // Current modifier value for xCoord
    private double yModifier; // Current modifier value for yCoord

    private MotorState jointAState; // used for keeping track of the motor state
                                    // for arm A
    private MotorState jointBState; // used for keeping track of the motor state
                                    // for arm B

    double aAngleVal = 0;
    double bAngleVal = 0;

    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;
    private ArmStatus status;
    
    protected UtilityArm() {
        DataServerWebClient client = new DataServerWebClient("http://172.22.11.1:8083");
        status = new ArmStatus(client);
        try {
            client.pushSchema(status.getSchema());
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        aP = .5; // 2.7
        aI = 0.00035; // 0.00076
        aD = 50; // 2
        bP = .5; // 3.5
        bI = 0.00065; // 0.0001
        bD = 50; // 2

        robotControl = RobotControlWithSRX.getInstance();

        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A)
                .setFeedbackDevice(FeedbackDevice.QuadEncoder);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B)
                .setFeedbackDevice(FeedbackDevice.QuadEncoder);

        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A)
                .changeControlMode(TalonControlMode.Position);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B)
                .changeControlMode(TalonControlMode.Position);

        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A).setPID(aP, aI,
                aD);
        robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B).setPID(bP, bI,
                bD);

        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        RESET_A_POSITION = SensorInputControlSRX
                .getInstance().INITIAL_POT_A_POSITION;
        RESET_B_POSITION = SensorInputControlSRX
                .getInstance().INITIAL_POT_B_POSITION;
        jointAPosition = RESET_A_POSITION; // Reset Position
        jointBPosition = RESET_B_POSITION; // Reset Position
        xCoord = -1; // Reset Coordinate
        yCoord = 4; // Reset Coordinate
        driverInputControl = DriverInputControlSRX.getInstance();
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
        xCoord += xModifier;
        yCoord -= yModifier;

        xModifier = yModifier = 0;

        if (Math.abs(driverInputControl.getControllerThrottle()) > DEAD_ZONE_X
                || Math.abs(driverInputControl
                        .getControllerTwist()) > DEAD_ZONE_Y) {
            if (Math.abs(
                    driverInputControl.getControllerThrottle()) > DEAD_ZONE_X) {
                xModifier = driverInputControl.getControllerThrottle()
                        * INCREMENT_RATE;
            }
            if (Math.abs(
                    driverInputControl.getControllerTwist()) > DEAD_ZONE_Y) {
                yModifier = driverInputControl.getControllerTwist()
                        * INCREMENT_RATE;
                // Up on joystick gives negative values
            }
            goTo(xCoord, yCoord);
            status.setDestX(xCoord);
            status.setDestY(yCoord);
            status.setAlpha(jointADegree);
            status.setBeta(jointBDegree);
            try {
                status.push();
            } catch (IOException e) {
                DriverStation.reportError("CANT CONNECT", false);
            }
        }

        if (driverInputControl.isResetButtonDown()) {
            resetPos();
            DriverStation.reportError("\nReseting...", false);
        }

        // DriverStation.reportError(
        // "X Coordinate: " + xCoord + " --- Y Coordinate: " + yCoord,
        // false);
    }

    @Override
    public void updateOutputs() {
        DriverStation.reportError(
                "\n Moving to JointAPosition: " + jointAPosition
                        + " --- Moving to JointBPosition: " + jointBPosition,
                false);
        robotControl.updateArmMotors(jointAPosition, jointBPosition);
    }

    /**
     * A simple down to earth equation for calculating the angles required to
     * reach a point.
     * 
     * @see <a href=
     *      "https://docs.google.com/drawings/d/1dR8t4AcUfh1KOq0hK-Sh3xFV05fgMqhl_l8BH47n6JQ/edit?usp=sharing"
     *      >Arm Constraints, explained</a>
     * 
     * @param xEndPoint
     *            the x coordinate of the new end-point
     * @param yEndPoint
     *            the y coordinate of the new end-point
     */
    public void goTo(double xEndPoint, double yEndPoint) {
        /*
         * Explanation of if-statements (exact numbers may change):
         * https://docs.google.com/drawings/d/1dR8t4AcUfh1KOq0hK-
         * Sh3xFV05fgMqhl_l8BH47n6JQ/edit?usp=sharing
         */

        if (xEndPoint < -BOUNDARY - FRAME_LENGTH) {
            xEndPoint = -BOUNDARY - FRAME_LENGTH;
        }
        if (xEndPoint > X_MAX_BACK_REACH) {
            xEndPoint = X_MAX_BACK_REACH;
        }
        if (yEndPoint > Y_MAX_UP_REACH) {
            yEndPoint = Y_MAX_UP_REACH;
        }
        if (yEndPoint < Y_MAX_DOWN_REACH) {
            yEndPoint = Y_MAX_DOWN_REACH;
        }
        if (yEndPoint < 3 && xEndPoint > -FRAME_LENGTH) {
            xEndPoint = -5;
        }

        if (xEndPoint < 1 && xEndPoint > -1 && yEndPoint < 6) {
            yEndPoint = 6;
        }

        if (xEndPoint >= 1 / 45.0 && yEndPoint < 22
                && yEndPoint < Math.sqrt(45 * (xEndPoint + 4.0 / 45)) + 1) {
            xEndPoint = Math.pow((yEndPoint - 1), 2) / 45.0 - 4.0 / 45;
        }

        if (yEndPoint > 28 && (yEndPoint) > Math
                .sqrt((1 - (xEndPoint * xEndPoint)
                        / (Math.pow((BOUNDARY + FRAME_LENGTH), 2))) * (5 * 5))
                + 28) {
            yEndPoint = (Math.sqrt((1 - (xEndPoint * xEndPoint)
                    / ((Math.pow((BOUNDARY + FRAME_LENGTH), 2)))) * (5 * 5)))
                    + 28;
        }

        xCoord = xEndPoint;
        yCoord = yEndPoint;

        // DriverStation.reportError(
        // "\n Going to: (" + xCoord + ", " + yCoord + ")", false);

        double p = Math.sqrt((xEndPoint * xEndPoint) + (yEndPoint * yEndPoint));
        double k = ((p * p) + LENGTH_A * LENGTH_A - LENGTH_B * LENGTH_B)
                / (2 * p);

        double x1 = (xEndPoint * k) / p
                + (yEndPoint / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);
        double y1 = (yEndPoint * k) / p
                - (xEndPoint / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);

        double x2 = (xEndPoint * k) / p
                - (yEndPoint / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);
        double y2 = (yEndPoint * k) / p
                + (xEndPoint / p) * Math.sqrt(LENGTH_A * LENGTH_A - k * k);

        double finaly = 0;
        double finalx = 0;
        if (y1 < 0) {
            finaly = y2;
            finalx = x2;
        } else {
            finaly = y1;
            finalx = x1;
        }

        jointADegree = Math.toDegrees(Math.atan2(finaly, finalx));

        double transformedX = (xEndPoint - finalx);
        double transformedY = (yEndPoint - finaly);
        jointBDegree = Math.toDegrees(Math.atan2(transformedY, transformedX));

        // DriverStation.reportError("\nJoint A Degree: " + jointADegree
        // + " --Joint B Degree: " + jointBDegree, false);

        jointBDegree = 180 - jointBDegree;

        if (jointBDegree > 350 - jointADegree) {
            jointBDegree = jointBDegree % 360 - 360;
        }

        if (jointBDegree < -jointADegree + 10) {
            jointBDegree = -jointADegree + 10;
        }

        jointBDegree += jointADegree;

        // DriverStation.reportError("\nJoint A Degree 2: " + jointADegree
        // + " --Joint B Degree 2: " + jointBDegree, false);

        jointAPosition = jointADegree * CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_A_POSITION;
        jointBPosition = -1 * jointBDegree * CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_B_POSITION;

        // DriverStation
        // .reportError(
        // "\nJoint A Position: " + jointAPosition
        // + " --Joint B Position: " + jointBPosition
        // + "Initial B Pot: "
        // + SensorInputControlSRX
        // .getInstance().INITIAL_POT_B_POSITION,
        // false);
    }

    /**
     * Method for autonomous use to determine if the arm has moved to position
     * 
     * @return Returns boolean whether or not the arm has reached the proper
     *         angles within a margin of error
     */
    public boolean isFinished() {
        boolean isJointAFinished = Math.abs(
                robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A).get()
                        - jointAPosition) < POSITION_MARGIN_ERR;
        boolean isJointBFinished = Math.abs(
                robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B).get()
                        - jointBPosition) < POSITION_MARGIN_ERR;
        // DriverStation.reportError(
        // "\nisFinished: " + (isJointAFinished && isJointBFinished),
        // false);
        return isJointAFinished && isJointBFinished;
    }

    public void resetPos() {
        jointAPosition = RESET_A_POSITION;
        jointBPosition = RESET_B_POSITION;
        xCoord = -1; // An approximation
        yCoord = 4; // An approximation
    }

    public double getCurrentDegreeA() {
        double degree = 0;
        degree = (robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A).get()
                + RESET_A_POSITION) / CONVERSION_FACTOR;
        return degree;
    }

    public double getCurrentDegreeB() {
        double degree = 0;
        degree = (robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B).get()
                + RESET_B_POSITION) / CONVERSION_FACTOR;
        return degree;
    }

}
