package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class ActiveIntake implements Module {

    public static final double INTAKE_SPEED = -1;
    private static ActiveIntake instance;
    private double intakeSpeed;
    private DoubleSolenoid.Value isIntaking;
    private MotorState intakeState;
    private DriverInputControlSRX driverInputControl;
    private RobotControlWithSRX robotControl;
    private double counter;
    private boolean previousIntakeToggle;
    private static final double delay = 1000;
    public static final DoubleSolenoid.Value intakeUp = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value intakeDown = DoubleSolenoid.Value.kReverse;

    private ActiveIntake() {
        driverInputControl = DriverInputControlSRX.getInstance();
        robotControl = RobotControlWithSRX.getInstance();

        isIntaking = intakeUp;
        reset();
        previousIntakeToggle = false;
        DriverStation.reportError("" + isDown(), false);
    }
    public static ActiveIntake getInstance() {
        if (instance == null) {
            instance = new ActiveIntake();
        }
        return instance;
    }
    public void init(){
        
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
                    DriverStation.reportError("\nIntaking", false);
        }

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_OUT))) {
            intakeState = MotorState.FORWARD;
            intakeSpeed = INTAKE_SPEED;
        }
//        DriverStation.reportError("Solenoid Button State " + driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID) 
//        + "\n", false);
//        DriverStation.reportError("  Solenoid State " + isIntaking + "\n", false);
        if (driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID)
                && !previousIntakeToggle ) {
            isIntaking = isIntaking == intakeUp ? intakeDown : intakeUp;
        }
        previousIntakeToggle = driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID);
        if(Shooter.getInstance().getRelativeTilt() > Shooter.LOWER_TILT_COLLISION && Shooter.getInstance().getRelativeTilt() < Shooter.UPPER_TILT_COLLISION){
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
//        DriverStation.reportError("\nIntake Motor Speed " + intakeSpeed + "\nSolenoid State" + isIntaking, false);
        robotControl.updateIntakeMotor(intakeSpeed);
        robotControl.updateDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER,
                isIntaking);
        
    }
    @Override
    public void update() {
        updateIntake();
    }
    public void setIntakeSolenoid(DoubleSolenoid.Value input){
//        DriverStation.reportError("\n Intake Position" + input, false);
        isIntaking = input;
    }
    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }
    public boolean isDown(){
        return isIntaking == intakeDown;
    }
}