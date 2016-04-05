package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class UtilityArm implements Module {

    /*
     * the positions - 1 for reset only - rest for cycling
     * 
     * calling encoder stuff
     */

    private int currCyclePos;
    private boolean preIncUp;

    private static int[] positions = { 0 }; // in degree
    private double movementDirection;

    private final double CONVERSION_FACTOR = 4096 / 360;

    private final double ARM_P = 1.0;
    private final double ARM_I = 0;
    private final double ARM_D = 0;
    
    private final double ARM_P_DOWN = 1.0;
    private final double ARM_I_DOWN = 0;
    private final double ARM_D_DOWN = 0;

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
        movementDirection = 1.0;
    }

    @Override
    public void update() {
        double oldCyclePos = currCyclePos;
        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_RESET)) {
            currCyclePos = 0;
        }

        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE) && !preIncUp) {
            currCyclePos = currCyclePos > positions.length ? 1 : currCyclePos + 1;
        }
        preIncUp = DriverInputControlSRX.getInstance().getButton(RobotButtonType.UTILITY_ARM_CYCLE);
        
        movementDirection = oldCyclePos < currCyclePos ? 1.0: -1.0;
    }

    @Override
    public void updateOutputs() {
        if(movementDirection < 0){
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setPID(ARM_P_DOWN, ARM_I_DOWN, ARM_D_DOWN);
        } else{
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).setPID(ARM_P, ARM_I, ARM_D);
        }
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.UTILITY_ARM).set(positions[currCyclePos] * CONVERSION_FACTOR);
    }

    @Override
    public void init() {}

}