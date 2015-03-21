package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

import edu.wpi.first.wpilibj.Joystick;

public class DriverInputControl {
    private static DriverInputControl instance = null;

    private HashMap<RobotJoystickType, Joystick> joystickMap;

    public static final double DEADZONE = 0.1;
    public static final double MIN_TORQUE_RESPONSE = 0.2;
    public static final double JOYSTICK_FULL_POWER_BAND = 0.9;

    protected DriverInputControl() {
        joystickMap = new HashMap<RobotJoystickType, Joystick>();
    }
    public static DriverInputControl getInstance() {
        if (instance == null) {
            instance = new DriverInputControl();
        }
        return instance;
    }
    public void addJoystick(RobotJoystickType type, int port) {
        joystickMap.put(type, new Joystick(port));
    }

    public static double deadzone(double axis) {
        if (Math.abs(axis) < DEADZONE) {
            return 0;
        }
        return axis;
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
    public double getLeftDrive() {
        double axis = joystickMap.get(RobotJoystickType.LEFT_DRIVE).getAxis(
                Joystick.AxisType.kY);
        return (DrivetrainControl.getInstance().getGearState() == GearState.LOW_GEAR ? deadzone(axis) : deadzone(axis) );
    }
    public double getRightDrive() {
        double axis = joystickMap.get(RobotJoystickType.RIGHT_DRIVE).getAxis(
                Joystick.AxisType.kY);
        return (DrivetrainControl.getInstance().getGearState() == GearState.LOW_GEAR ? deadzone(axis) : deadzone(axis) );
    }
    public Joystick getJoystick(RobotJoystickType joystickType) {
        return joystickMap.get(joystickType);
    }
    public boolean getButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
                .getInstance().getButtonMap().get(buttonType);
        return getJoystick(buttonMatch.getJoystickType()).getRawButton(
                buttonMatch.getPort());
    }

    public int getPOVButton(RobotButtonType buttonType) {
    	JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
				.getInstance().getButtonMap().get(buttonType);
    		
		int povValue = getJoystick(buttonMatch.getJoystickType()).getPOV(buttonMatch.getPort());
		
		if(povValue > -1) {
			return povValue;
		}
    	
		return -1;
    }

    public double getPressureButton(RobotButtonType buttonType){
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap
                .getInstance().getButtonMap().get(buttonType);
        return getJoystick(buttonMatch.getJoystickType()).getRawAxis(
                buttonMatch.getAxisType().value);
    }
}