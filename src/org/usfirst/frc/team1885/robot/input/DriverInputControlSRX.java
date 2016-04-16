package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

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
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap.getInstance().getButtonMap().get(buttonType);
        if (buttonMatch != null) {
            return getJoystick(buttonMatch.getJoystickType()).getRawButton(buttonMatch.getPort());
        }
        return false;
    }

    public int getPOVButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap.getInstance().getButtonMap().get(buttonType);

        int povValue = getJoystick(buttonMatch.getJoystickType()).getPOV(buttonMatch.getPort());

        if (povValue > -1) {
            return povValue;
        }

        return -1;
    }

    public double getPressureButton(RobotButtonType buttonType) {
        JoystickButtonMatch buttonMatch = (JoystickButtonMatch) JoystickButtonMap.getInstance().getButtonMap().get(buttonType);
        double rawAxis = 0;
        try {
            rawAxis = getJoystick(buttonMatch.getJoystickType()) .getRawAxis(buttonMatch.getAxisType().value);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return rawAxis;
    }

    public void update() {
        this.update(
                this.getJoystick(RobotJoystickType.LEFT_DRIVE) .getAxis(AxisType.kY),
                this.getJoystick(RobotJoystickType.RIGHT_DRIVE) .getAxis(AxisType.kY));
    }
    public void update(double leftJoystick, double rightJoystick) {
        this.rightDriveSpeed = deadzone(rightJoystick);
        this.leftDriveSpeed = deadzone(leftJoystick);
        DrivetrainControl.getInstance().setLeftDriveSpeed(leftDriveSpeed);
        DrivetrainControl.getInstance().setRightDriveSpeed(rightDriveSpeed);
    }
    public double getControllerTwist() {
        return this.getJoystick(RobotJoystickType.CONTROLLER).getAxis(AxisType.kTwist);
    }

    public double getControllerThrottle() {
        return this.getJoystick(RobotJoystickType.CONTROLLER).getAxis(AxisType.kThrottle);
    }

    public boolean isButtonDown(RobotButtonType type) {
        return this.getButton(type);
    }
    public int getShooterTilt() {
        if(this.isButtonDown(RobotButtonType.SHOOTER_TILT_UP)) {
            return 1;
        }
        if(this.isButtonDown(RobotButtonType.SHOOTER_TILT_DOWN)){
            return -1;
        }
        return 0;
    }
    public double getShooterTwist() {
        if(this.isButtonDown(RobotButtonType.SHOOTER_TWIST_RIGHT)) {
            return Shooter.TWIST_SPEED;
        }
        if(this.isButtonDown(RobotButtonType.SHOOTER_TWIST_LEFT)){
            return -Shooter.TWIST_SPEED;
        }
        if(this.isButtonDown(RobotButtonType.SHOOTER_PAN_RIGHT_MANUAL)){
            return Shooter.TWIST_SPEED;
        }
        if(this.isButtonDown(RobotButtonType.SHOOTER_PAN_LEFT_MANUAL)){
            return -Shooter.TWIST_SPEED;
        }
        return 0;
    }
    public boolean isResetButtonDown() {
        return this.isButtonDown(RobotButtonType.RESET_AUX_ARM);
    }
}
