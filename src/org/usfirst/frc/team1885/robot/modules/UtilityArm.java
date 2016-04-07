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
import edu.wpi.first.wpilibj.DriverStation;

public class UtilityArm implements Module {

    /*
     * the positions - 0 for reset only - rest for cycling
     */

    private final double ERROR_MARGIN = 75.0; //encoder ticks
    
    private double relativeAngle;
    private int currCyclePos;
    private boolean preIncUp;
    private double power;

    private static int[] positions = { 0 }; // in degree
                                            // First value is reset Pos
    
    private final double POWER = 0.6;
    private final double CONVERSION_FACTOR = 4096.0 / 360.0;
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
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setPID(ARM_P, ARM_I, ARM_D);
        relativeAngle = 0;
        power = 0;
    }

    @Override
    public void update() {
        power = 0;
        
        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_RESET)) {
            currCyclePos = 0;
        }

        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE) && !preIncUp) {
            if(positions.length > 1){
                currCyclePos = currCyclePos >= positions.length - 1 ? 1 : currCyclePos + 1;
            }
        }
        preIncUp = DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE);
        
        this.relativeAngle = positions[currCyclePos];

        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() < (this.relativeAngle * CONVERSION_FACTOR)){
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.Speed);
            power = POWER;
        } else{
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).changeControlMode(TalonControlMode.Position);
        }
        if(SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.ARM_LIMITER).get()){
            power = 0;
            this.relativeAngle = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() / CONVERSION_FACTOR;
        }
        if(Shooter.getInstance().getRelativeTilt() > SHOOTER_COLLISION_DEGREE){
            currCyclePos = positions.length - 1;
            this.relativeAngle = positions[currCyclePos];
        }
    }
    
    public double boundPosition(double inputPosition){
        DriverStation.reportError("\n" + SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.ARM_LIMITER).get(), false);
        return SensorInputControlSRX.getInstance().getLimitSwitch(SensorType.ARM_LIMITER).get() ? RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() : inputPosition;
    }

    @Override
    public void updateOutputs() {
        DriverStation.reportError("\nIntended Angle: " + this.relativeAngle + "  Current Angle: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition() / CONVERSION_FACTOR, false);
        DriverStation.reportError("\nPower:: " + this.power, false);
        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getControlMode().equals(TalonControlMode.Position)){
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).set(boundPosition(this.relativeAngle * CONVERSION_FACTOR));
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
        return ((this.relativeAngle * CONVERSION_FACTOR) - (RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).getEncPosition())) >= ERROR_MARGIN;
    }

}
