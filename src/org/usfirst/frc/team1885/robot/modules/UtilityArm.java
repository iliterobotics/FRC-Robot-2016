package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.common.type.UtilityArmPosition;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class UtilityArm implements Module {

    private final double ERROR_MARGIN = 75.0; //encoder ticks
    private final double FLOOR_POSITION = 148.0;
    private final double DRAWBRIDGE_POSITION = FLOOR_POSITION - 40;
    private final double ZERO = 20; //encoder ticks
    
    private double position;
    private int currCyclePos;
    private boolean preIncUp;
    private double power;

    private double[] positions = { 0, 254, 875, 1429 }; // in degree // First value is reset position
    
    public static final double POWER_DOWN = 0.3;
    public static final double POWER_UP = -0.4;
    private final double CONVERSION_FACTOR = 1024.0 / 360.0;
    private static final double SHOOTER_COLLISION_DEGREE = 100;

    private final double ARM_P = 1.0;
    private final double ARM_I = 0;
    private final double ARM_D = 0;

    private static UtilityArm instance;

    public static UtilityArm getInstance() {
        if (instance == null) {
            instance = new UtilityArm();
        }
        return instance;
    }

    public UtilityArm() {
        //Currently in percent power mode, capabl of changing to position mode if we test PID
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.PercentVbus);
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setPID(ARM_P, ARM_I, ARM_D);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).reverseSensor(true);
        position = 0;
        power = 0;
    }

    @Override
    public void update() {
        power = 0;
        
        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_RESET)) {
            currCyclePos = 0;
            power = POWER_UP;
        }

        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE) && !preIncUp) {
            if(positions.length > 1){
                currCyclePos = currCyclePos >= positions.length - 1 ? 1 : currCyclePos + 1;
            }
            power = POWER_DOWN;
        }
        
        if(DriverInputControlSRX.getInstance().getPOVButton(RobotButtonType.AIM) == 180){
            currCyclePos = 0;
            power = POWER_UP;
        }
//        preIncUp = DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE);
        
        this.position = positions[currCyclePos];

        if(Shooter.getInstance().getRelativeTilt() > SHOOTER_COLLISION_DEGREE){
            currCyclePos = positions.length - 1;
            this.position = positions[currCyclePos];
            power = POWER_DOWN;
        } else if(currCyclePos == 0){
            power = POWER_UP;
        }
        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getControlMode().equals(TalonControlMode.PercentVbus) && RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() <= ZERO){
            power = power < 0 ? 0 : power;
        }
        if(!SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.ARM_LIMITER).get()){
            if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() != this.position){
                RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.PercentVbus);
            }
            this.position = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition();
            power = power > 0 ? 0 : power;
        }
    }
    
    public double boundPosition(double inputPosition){
        return SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.ARM_LIMITER).get() ? RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() : inputPosition;
    }

    @Override
    public void updateOutputs() {
        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getControlMode().equals(TalonControlMode.Position)){
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).set(this.position);
        } else{
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).set(this.power);
        }
    }

    @Override
    public void init() {}
    
    public boolean setPosition(UtilityArmPosition position){
        switch(position){
            case RESET: currCyclePos = 0; break;
            case POS_1: currCyclePos = 1; break;
            case POS_2: currCyclePos = 2; break;
        }
        return ((this.position * CONVERSION_FACTOR) - (RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition())) >= ERROR_MARGIN;
    }
    
    public double setPower(double power){
        this.power = power;
        return this.power;
    }
}
