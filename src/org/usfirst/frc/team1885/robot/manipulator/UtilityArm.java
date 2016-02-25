package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

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

    public static final double LENGTH_A = 17.5; // length of arm A
    public static final double LENGTH_B = 18; // length of arm B
    public static final double CONVERSION_FACTOR = 1024 / 360.0; // multiplier to convert from degrees to ticks
    public static final double DEF_A_ANGLE = 10 / CONVERSION_FACTOR, // Starting tick amount of arm A
                               DEF_B_ANGLE = 173 / CONVERSION_FACTOR; // Starting tick amount of arm B
    private static final double DEGREE_MARGIN_ERR = 2 / CONVERSION_FACTOR;

    private double jointAPosition; // storage for updating the A angle
    private double jointBPosition; // storage for updating the B angle
    private double aP, aI, aD; // values for the PID to move joint A
    private double bP, bI, bD; // values for the PID to move joint B
    private double x, y; // x and y positions that arm is moving towards or at


    private MotorState jointAState; // used for keeping track of the motor state for arm A
    private MotorState jointBState; // used for keeping track of the motor state for arm B
    
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
        if (driverInputControl.isResetButtonDown()) {
            resetPos();
        }
        //joystick stuff to put in later
        //double joystick_x = ;
        //double joystick_y = ;
        //goTo(x + joystick_x, y + joysick_y);
        updateOutputs();
    }

    @Override
    public void updateOutputs() {
        robotControl.updateArmMotors(jointAPosition, jointBPosition);
    }

    /**
     * A simple down to earth equation for calculating the angles required to reach a point
     * 
     * @param x
     *            the x coordinate of the new end-point
     * @param y
     *            the y coordinate of the new end-point
     */
    public void goTo(double x, double y) {

        this.x = x;
        this.y = y;
        
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

        jointAPosition = Math.toDegrees(Math.atan2(finaly, finalx));

        double transformedX = (x - finalx);
        double transformedY = (y - finaly);
        jointBPosition = Math.toDegrees(Math.atan2(transformedY, transformedX));


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



        jointAPosition = jointAPosition / CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_A_POSITION;
        jointBPosition = jointBPosition / CONVERSION_FACTOR
                + SensorInputControlSRX.getInstance().INITIAL_POT_B_POSITION;
    }
    
    /**
     * Method for autonomous use to determine if the arm has moved to position
     * @return Returns boolean whether or not the arm has reached the proper angles within a margin of error
     */
    public boolean isFinished() {
        boolean isJointAFinished = Math.abs(robotControl.getTalons().get(RobotMotorType.ARM_JOINT_A).get() 
                                    - jointAPosition) < DEGREE_MARGIN_ERR;
        boolean isJointBFinished = Math.abs(robotControl.getTalons().get(RobotMotorType.ARM_JOINT_B).get()
                                    - jointBPosition) < DEGREE_MARGIN_ERR;
        return isJointAFinished && isJointBFinished;
    }
    
    public void resetPos() {
        jointAPosition = DEF_A_ANGLE;
        jointBPosition = DEF_B_ANGLE;
    }

}
