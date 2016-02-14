package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class ActiveIntake implements Module {

    public static final double INTAKE_SPEED = .5;
    private static ActiveIntake instance;
    private double intakeSpeed;
    private MotorState intakeState;
    private DriverInputControlSRX driverInputControl;

    protected ActiveIntake() {
        this.intakeState = MotorState.OFF;
        intakeSpeed = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
    }
    public static ActiveIntake getInstance() {
        if (instance == null) {
            instance = new ActiveIntake();
        }
        return instance;
    }
    public void setMotorState(MotorState intakeState) {
        this.intakeState = intakeState;
    }
    public MotorState getIntakeMotorState() {
        return intakeState;
    }
    public double getIntakeSpeed() {
        return intakeSpeed;
    }
    public void updateIntake() {
        intakeSpeed = 0;

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_IN))) {
            intakeState = MotorState.REVERSE;
            intakeSpeed = -INTAKE_SPEED;
        }

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_OUT))) {
            intakeState = MotorState.FORWARD;
            intakeSpeed = INTAKE_SPEED;
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
    }

    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateIntakeMotor(intakeSpeed);
    }
    @Override
    public void update() {
        updateIntake();
        updateOutputs();
    }
}