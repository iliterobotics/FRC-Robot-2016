package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class ActiveIntake implements Module {

    public static final double INTAKE_SPEED = 1;
    private static ActiveIntake instance;
    private double intakeSpeed;
    private boolean isIntaking;
    private MotorState intakeState;
    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;
    private double counter;
    private static final double delay = 1000;

    protected ActiveIntake() {
        this.intakeState = MotorState.OFF;
        intakeSpeed = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
        robotControl = RobotControlWithSRX.getInstance();
        isIntaking = false;
        counter = System.currentTimeMillis();
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
            // DriverStation.reportError("Suck It\n", false);
        }

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_OUT))) {
            intakeState = MotorState.FORWARD;
            intakeSpeed = INTAKE_SPEED;
            // DriverStation.reportError("Spit Out\n", false);
        }
        if ((driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID)
                && System.currentTimeMillis() >= counter + delay)) {
            isIntaking = !isIntaking;
            counter = System.currentTimeMillis();
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
        robotControl.updateIntakeMotors(intakeSpeed);
        robotControl.updateSingleSolenoid(RobotPneumaticType.INTAKE_SETTER,
                isIntaking);
    }
    @Override
    public void update() {
        updateIntake();
        updateOutputs();
    }
}