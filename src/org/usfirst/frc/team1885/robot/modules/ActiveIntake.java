package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ActiveIntake implements Module {

    public static final double INTAKE_SPEED = 1;
    private static ActiveIntake instance;
    private double intakeSpeed;
    private DoubleSolenoid.Value isIntaking;
    private MotorState intakeState;
    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;
    private boolean previousToggleState;
    private boolean previousLowState;
    private boolean previousHighState;

    protected ActiveIntake() {
        previousHighState = false;
        previousLowState = false;
        previousToggleState = false;
        driverInputControl = DriverInputControlSRX.getInstance();
        robotControl = RobotControlWithSRX.getInstance();

        isIntaking = DoubleSolenoid.Value.kOff;
        reset();
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
                && !previousToggleState)) {
            isIntaking = isIntaking == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;
        }
        previousToggleState = driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID);
        if (driverInputControl.getButton(RobotButtonType.READY_LOW)
                && !previousLowState) {
            robotControl.updateSingleSolenoid(RobotPneumaticType.INTAKE_SETTER, true);
        }
        previousLowState = driverInputControl.getButton(RobotButtonType.READY_LOW);
        if (driverInputControl.getButton(RobotButtonType.READY_HIGH)
                && !previousHighState) {
            robotControl.updateSingleSolenoid(RobotPneumaticType.INTAKE_SETTER, false);
        }
        previousHighState = driverInputControl.getButton(RobotButtonType.READY_HIGH);
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
        isIntaking = DoubleSolenoid.Value.kForward;        
        updateOutputs();
    }
    public void updateOutputs() {
        robotControl.updateIntakeMotors(intakeSpeed);
        robotControl.updateDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER,
                isIntaking);
    }
    @Override
    public void update() {
        updateIntake();
        updateOutputs();
    }
    public void intakeUp(){
       isIntaking = DoubleSolenoid.Value.kForward;
    }
    public void intakeDown(){
        isIntaking = DoubleSolenoid.Value.kReverse;
    }
}