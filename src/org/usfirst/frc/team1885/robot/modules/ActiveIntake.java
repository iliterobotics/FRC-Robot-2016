package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class ActiveIntake implements Module {

    private static ActiveIntake instance;
    
    public static final double INTAKE_SPEED = -1;
    public static final DoubleSolenoid.Value intakeUp = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value intakeDown = DoubleSolenoid.Value.kReverse;
    
    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;
    
    private double intakeSpeed;
    private MotorState intakeState;
    private DoubleSolenoid.Value isIntaking;
    private boolean previousIntakeToggle;

    private ActiveIntake() {
        driverInputControl = DriverInputControlSRX.getInstance();
        robotControl = RobotControlWithSRX.getInstance();

        isIntaking = intakeUp;
        reset();
        previousIntakeToggle = false;
    }
    public static ActiveIntake getInstance() {
        if (instance == null) {
            instance = new ActiveIntake();
        }
        return instance;
    }
    public void init() {}

    public MotorState getIntakeMotorState() {
        return intakeState;
    }
    public double getIntakeSpeed() {
        return intakeSpeed;
    }
    public void update() {
        intakeSpeed = 0;
        if ((driverInputControl.getButton(RobotButtonType.INTAKE_IN))) {
            intakeState = MotorState.REVERSE;
            intakeSpeed = -INTAKE_SPEED;
            DriverStation.reportError("\nIntaking", false);
        }

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_OUT))) {
            intakeState = MotorState.FORWARD;
            intakeSpeed = INTAKE_SPEED;
        }

        if (driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID) && !previousIntakeToggle) {
            isIntaking = isIntaking == intakeUp ? intakeDown : intakeUp;
        }
        previousIntakeToggle = driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID);

        if (Shooter.getInstance().getRelativeTilt() > Shooter.LOWER_TILT_COLLISION && Shooter.getInstance().getRelativeTilt() < Shooter.UPPER_TILT_COLLISION) {
            isIntaking = intakeDown;
        }

        updateIntake(intakeSpeed);
    }
    public void updateIntake(double intakeSpeed) {
        if (intakeSpeed > 0) {
            intakeState = MotorState.FORWARD;
        } else if (intakeSpeed < 0) {
            intakeState = MotorState.REVERSE;
        } else {
            intakeState = MotorState.OFF;
        }
    }

    public void reset() {
        this.intakeState = MotorState.OFF;
        intakeSpeed = 0;
        isIntaking = intakeUp;
        updateOutputs();
    }

    public void updateOutputs() {
        robotControl.updateIntakeMotor(intakeSpeed);
        robotControl.updateDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER, isIntaking);
    }

    public void setIntakeSolenoid(DoubleSolenoid.Value input) {
        isIntaking = input;
    }
    public void setIntakeSpeed(double speed) {
        intakeSpeed = speed;
    }
    public boolean isDown() {
        return isIntaking == intakeDown;
    }
}