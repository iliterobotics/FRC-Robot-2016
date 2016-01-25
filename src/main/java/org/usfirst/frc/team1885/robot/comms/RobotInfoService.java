package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.graveyard.ActiveIntake;
import org.usfirst.frc.team1885.graveyard.ClawControl;
import org.usfirst.frc.team1885.graveyard.RobotControl;
import org.usfirst.frc.team1885.graveyard.SensorInputControl;
import org.usfirst.frc.team1885.graveyard.ToteLift;
import org.usfirst.frc.team1885.robot.common.type.GearState;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class RobotInfoService {

	private static RobotInfoService instance;
	private RobotInfoMessage rim = new RobotInfoMessage();
	private DrivetrainControl dtcInstance = DrivetrainControl.getInstance();
	private SensorInputControl sicInstance = SensorInputControl.getInstance();
	private RobotControl rcInstance = RobotControl.getInstance();
	private ToteLift tlInstance = ToteLift.getInstance();
	private ClawControl ccInstance = ClawControl.getInstance();
	private ActiveIntake aiInstance = ActiveIntake.getInstance();

	private int liftEncoderTicks;
	private boolean stabilizer, firstHookAtBottom, lowPressure;
	private String gearState;
	private double powerLevel, toteDistance; // Remember to add the >9000 easter egg thing :D
	private MotorState activeStateLeft;
	private MotorState activeStateRight;

	public static RobotInfoService getInstance() {
		if (instance == null)
			instance = new RobotInfoService();
		return instance;
	}

	public void updateActiveIntake(){
	    activeStateLeft = aiInstance.getLeftMotorState();
	    activeStateRight = aiInstance.getRightMotorState();
	}
	
	public void updatePowerLevel() {
		PowerDistributionPanel pdp = new PowerDistributionPanel();
		powerLevel = pdp.getTotalPower();
	}

	private void updateEncoderTicks() {
		liftEncoderTicks = sicInstance.getEncoderTicks(SensorType.TOTE_ENCODER);
	}

	private void updateLowPressure() {
		lowPressure = rcInstance.getCompressor().getPressureSwitchValue();
	}

	private void updateMagnetSensor() {
		firstHookAtBottom = sicInstance
				.getLimitSwitch(SensorType.MAGNET_SENSOR).get();
	}

	private void updateToteDistance() {
		double tickDistance = tlInstance.getDistanceTraveled();
		// toteDistance = -enter distance calculation to cm here-
	}

	private void updateGearState() {
		if (dtcInstance.getGearState() == GearState.HIGH_GEAR)
			gearState = "High Gear";
		else if (dtcInstance.getGearState() == GearState.LOW_GEAR)
			gearState = "Low Gear";
	}

	private void updateStabilizer() {
		stabilizer = rcInstance.getSolenoids(RobotPneumaticType.TOTE_LIFT_STOP); // Confirm
	}
	
	public MotorState getActiveIntakeMotorStateLeft(){
	    return activeStateLeft;
	}
	
	public MotorState getActiveIntakeMotorStateRight(){
        return activeStateRight;
    }

	public int getLiftEncoderTicks() {
		return liftEncoderTicks;
	}

	public boolean isStabilizer() {
		return stabilizer;
	}

	public boolean isFirstHookAtBottom() {
		return firstHookAtBottom;
	}

	public boolean isLowPressure() {
		return lowPressure;
	}

	public String getGearState() {
		return gearState;
	}

	public double getPowerLevel() {
		return powerLevel;
	}

	public double getToteDistance() {
		return toteDistance;
	}
}

/*
 * Things we need to take from Robot.java and store/give to gui/drivers
 * 
 * - Update the FRC-Vision page on github
 * 
 * - Encoder ticks for the lift (DONE) - ToteLift distance (DONE) (Distance
 * given back in ticks) - Current speed (low priority) - Where the first hook is
 * (Bottom / Top) * (DONE) - Gear (low/high) * (DONE) - Which hook is currently
 * at the bottom * :: Have int positions for specific places, default to a
 * single value - If the hook is at the magnet (proximity to the reset value) *
 * - Distance from ramp (vision, maybe) - Distance from white tape thing
 * (vision, maybe) - Tote stabilizer solenoid position (DONE) - Battery life
 * (DONE) - Pressure sensor for pneumatics(Low Pressure/Not Low Pressure) (DONE)
 */