package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
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
    private DoubleSolenoid.Value previousState;
    private DoubleSolenoid.Value realityCheck;
    private final long REALITY_TIME = 2000;
    private long realityTimer;

    private ActiveIntake() {
        driverInputControl = DriverInputControlSRX.getInstance();
        robotControl = RobotControlWithSRX.getInstance();

        isIntaking = intakeUp;
        reset();
        previousIntakeToggle = false;
        DriverStation.reportError("" + isDown(), false);
        realityCheck = isIntaking;
        previousState = isIntaking;
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
//                    DriverStation.reportError("\nIntaking", false);
        }

        if ((driverInputControl.getButton(RobotButtonType.INTAKE_OUT))) {
            intakeState = MotorState.FORWARD;
            intakeSpeed = INTAKE_SPEED;
        }
//        DriverStation.reportError("Solenoid Button State " + driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID) 
//        + "\n", false);
//        DriverStation.reportError("  Solenoid State " + isIntaking + "\n", false);
        if (driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID) && !previousIntakeToggle ) {
//            previousState = isIntaking;
            isIntaking = isIntaking == intakeUp ? intakeDown : intakeUp;
        }
//        if(previousState == intakeUp && isIntaking == intakeDown){
//            if(System.currentTimeMillis() - realityTimer > REALITY_TIME){
//                realityCheck = intakeDown;
//            }
//        } else{
//            realityTimer = System.currentTimeMillis();
//            realityCheck = intakeUp;
//        }
        previousIntakeToggle = driverInputControl.getButton(RobotButtonType.INTAKE_SOLENOID);
        
//        if(Shooter.getInstance().getTilt() > Shooter.LOWER_TILT_COLLISION && Shooter.getInstance().getTilt() < Shooter.UPPER_TILT_COLLISION){
//            isIntaking = intakeDown;
//        }
        
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
        robotControl.updateDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER, isIntaking);  
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
//        DriverStation.reportError(  "\n" + SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.INTAKE_DOWN).get(), false);
//        return !SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.INTAKE_DOWN).get();
        return isIntaking == intakeDown;
    }
}