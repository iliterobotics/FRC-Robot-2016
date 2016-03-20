package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class UtilityArmMini implements Module{

    private static UtilityArmMini instance;
    
    private final double DEAD_ZONE = .1;
    private final double SPEED_PROPORTION = 0.5;
    
    private double armSpeed;
    private double relativePosition;
    
    public static UtilityArmMini getInstance(){
        if(instance == null){
            instance = new UtilityArmMini();
        }
        return instance;
    }
    
    private UtilityArmMini(){
        armSpeed = 0;
        relativePosition = 0;
    }
    
    @Override
    public void init() {
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_JOINT_A).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_JOINT_A).changeControlMode(TalonControlMode.PercentVbus);
    }
    
    @Override
    public void update() {
        double driverInput = DriverInputControlSRX.getInstance().getControllerTwist();
        this.armSpeed = Math.abs(driverInput) > DEAD_ZONE ? driverInput : 0;
        this.relativePosition = RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_JOINT_A).getEncPosition() - SensorInputControlSRX.getInstance().getInitialPotAPostition();
        
        this.armSpeed = boundPosition(this.armSpeed);
        
        this.armSpeed *= SPEED_PROPORTION;
        //Debugging
        DriverStation.reportError("\nArm Speed:: " + this.armSpeed + " Arm Position: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_JOINT_A).getEncPosition() + " Relative Arm Position:: " + this.relativePosition, false);
    }
    
    private double boundPosition(double inputSpeed){
        return this.relativePosition > 0 ? inputSpeed: 0;
    }

    @Override
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_JOINT_A).set(armSpeed);
    }

}
