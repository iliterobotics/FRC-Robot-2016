package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class DriverInputControlSRX {
    private HashMap<RobotJoystickType, Joystick> joystickMap;
    private static DriverInputControlSRX instance = null;
    public static final double DEADZONE = .1;
    public static final double MIN_TORQUE_RESPONSE = 0.2;
    private double leftDriveSpeed, rightDriveSpeed;

    private DriverInputControlSRX() {
        joystickMap = new HashMap<RobotJoystickType, Joystick>();
        leftDriveSpeed = 0;
        rightDriveSpeed = 0;
    }
    public static DriverInputControlSRX getInstance() {
        if (instance == null) {
            instance = new DriverInputControlSRX();
        }
        return instance;
    }
    public static double deadzone(double axis) {
        if (Math.abs(axis) < DEADZONE) {
            return 0;
        }
        return axis;
    }
    public double getLeftDrive() {
        return this.leftDriveSpeed;
    }
    public double getRightDrive() {
        return this.rightDriveSpeed;
    }
    public Joystick getJoystick(RobotJoystickType joystickType) {
        return joystickMap.get(joystickType);
    }
    public boolean addJoystick(RobotJoystickType joystickType,
            Joystick joystick) {
        joystickMap.put(joystickType, joystick);
        return true;
    }
    public boolean getButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
                .getInstance().getButtonMap().get(buttonType);
        if (buttonMatch != null) {
            return getJoystick(buttonMatch.getJoystickType())
                    .getRawButton(buttonMatch.getPort());
        }
        return false;
    }

    public int getPOVButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
                .getInstance().getButtonMap().get(buttonType);

        int povValue = getJoystick(buttonMatch.getJoystickType())
                .getPOV(buttonMatch.getPort());

        if (povValue > -1) {
            return povValue;
        }

        return -1;
    }

    public double getPressureButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
                .getInstance().getButtonMap().get(buttonType);
        double rawAxis = 0;
        try {
            rawAxis = getJoystick(buttonMatch.getJoystickType())
                    .getRawAxis(buttonMatch.getAxisType().value);
        } catch (Exception e) {
            System.err.println("EXCEPTION on buttonType= " + buttonType);
            e.printStackTrace();
        }
        return rawAxis;
    }
    public static double expScale(double axis){
        double A = 1.85; // X^3 coefficient

        double B = -0.7; // X^2 coefficient

        // X coefficient - this correlates to the approximate linear increase during low-inputs (i.e. on the graph, 0.1 < x < 0.4).  This creates a "low power band" that is easy for the driver to find, where the robot responds at its minimum practical power to allow for fine-tuned movements
        double C = 0.2; 

        // Assume the variable Y_AXIS is represents the joystick axis input
        double ABS_Y = Math.abs(axis);
        double AXIS_SIGN = 1.0;
        if(axis < 0) AXIS_SIGN = -1.0;

        // Account for the Deadband, Max Power band, and Min Torque band, but exponentially scale it
        double T_D_EXP = 0; // default 0, only check for outside deadband cases
        if(ABS_Y >= DEADZONE){
          // Note - T_D_EXP is > 1.0 when ABS_Y > JOYSTICK_FULL_POWER_BAND, so no need to deal with that case specifically
          T_D_EXP = A*(Math.pow(ABS_Y-DEADZONE, 3)) + B*(Math.pow(ABS_Y-DEADZONE, 2)) + C*(ABS_Y-DEADZONE) + MIN_TORQUE_RESPONSE;
        }
        T_D_EXP = Math.min(T_D_EXP , 1.0) * AXIS_SIGN;
        return T_D_EXP;
    }

    public void update() {
        this.update(
                this.getJoystick(RobotJoystickType.LEFT_DRIVE)
                        .getAxis(AxisType.kY),
                this.getJoystick(RobotJoystickType.RIGHT_DRIVE)
                        .getAxis(AxisType.kY));
    }
    public void update(double leftJoystick, double rightJoystick)
    {    
        System.err.println("LEFT= " + leftJoystick +", RIGHT= " + rightJoystick);
        this.rightDriveSpeed = deadzone(rightJoystick);
        this.leftDriveSpeed = deadzone(leftJoystick);
        RobotControlWithSRX.getInstance().updateDriveSpeed(leftDriveSpeed, rightDriveSpeed);
    }
}
