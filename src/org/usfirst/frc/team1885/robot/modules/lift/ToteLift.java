package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class ToteLift implements Module{

	public static final double DEFAULT_LIFT_SPEED = -1.5;
	public static final double indexHeight = 1210;
	public static final double toteOffset = 1210 * .25;
//	public static final double DEFAULT_LIFT_SPEED_DOWN = .25;
	public static final double MAX_LIFT_SPEED_ENC = 4000;
	public static final double MAX_LIFT_SPEED_UP_ENC = -2500;
	public static final double MAX_LIFT_SPEED_DOWN_ENC = 1500;
	
	private static ToteLift instance;
	private double liftSpeed;
	private double desiredLiftSpeedTicks;
	private double motorSpeed;
	private MotorState state;
	private PID distanceControlLoop;
	private PID motorPowerControlLoop;
	private double distanceTraveled;
	private final double DEAD_ZONE = .15;
	private boolean isIncrementing;
	private boolean isBraked;
	private boolean isSupported;
	private boolean liftSupported = false;

	private boolean isResetting;

	private double liftIncrementHeight = 1210;
	private boolean isDecrementing;
	
	private boolean prevSupportState = false;
	
	private double relativeEncoder;

	protected ToteLift() {
		this.state = MotorState.OFF;
		distanceControlLoop = new PID(0.50, 0.005, 0);
		motorPowerControlLoop = new PID(.35, .1, 0);
		motorPowerControlLoop.setScalingValue(MAX_LIFT_SPEED_ENC);
		isBraked = false;
		liftSpeed = 0;
		isIncrementing = false;
		isDecrementing = false;
		relativeEncoder = 0;
	}

	public static ToteLift getInstance() {
		if (instance == null) {
			instance = new ToteLift();
		}
		return instance;
	}

	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	public void setMotorState(MotorState state) {
		this.state = state;
	}

	public MotorState getMotorState() {
		return state;
	}

	public double getSpeed() {
		return liftSpeed;
	}
	public boolean getIsBraked()
	{
		return isBraked;
	}
	public boolean getIsResetting()
	{
		return isResetting;
	}
	public void updateLift() {

		this.liftSpeed = 0;
		
		if ((DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_INCREMENT)
//				&&
//				!SensorInputControl.getInstance().isActive(
//		                SensorType.TOUCH_SENSOR_TOTE_RIGHT) &&
//                !SensorInputControl.getInstance().isActive(
//                        SensorType.TOUCH_SENSOR_TOTE_LEFT)
//				DriverInputControl
//				.getInstance().getButton(RobotButtonType.TOTE_LIFT_NUDGE_UP)
				)
				&& !isIncrementing || isIncrementing) {

			if (!isIncrementing) {
				this.reset();

				liftIncrementHeight = ToteLift.indexHeight + ToteLift.toteOffset;
				this.distanceControlLoop.setScalingValue(liftIncrementHeight);
//				if (DriverInputControl.getInstance().getButton(
//						RobotButtonType.TOTE_LIFT_NUDGE_UP)) {
//					liftIncrementHeight = 250;
//				}
			}

			this.isIncrementing = !incrementLift(liftIncrementHeight, 10);

		}

		if ((DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_DECREMENT) //|| DriverInputControl
//				.getInstance().getButton(RobotButtonType.TOTE_LIFT_NUDGE_DOWN)
				)
				&& !isDecrementing || isDecrementing) {

			if (!isDecrementing) {
				this.reset();

				liftIncrementHeight = -ToteLift.toteOffset;
				this.distanceControlLoop.setScalingValue(liftIncrementHeight);
//				if (DriverInputControl.getInstance().getButton(
//						RobotButtonType.TOTE_LIFT_NUDGE_DOWN)) {
//
//					liftIncrementHeight = -250;
//
//				}
			}

			this.isDecrementing = !incrementLift(liftIncrementHeight, 10);

		}

		if (DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_RESET)
				&& !isResetting || isResetting) {

			if (!isResetting) {
				this.reset();
				isResetting = true;
				liftIncrementHeight = -this.relativeEncoder;
				this.distanceControlLoop.setScalingValue(liftIncrementHeight);
			}
			
			this.isResetting = !incrementLift(liftIncrementHeight, 10);
			
//			liftSpeed = DEFAULT_LIFT_SPEED;
			if (!SensorInputControl.getInstance().isActive(
					SensorType.MAGNET_SENSOR)) {
				isResetting = false;
				liftSpeed = 0;
			}
		}

//		System.out.println(toString());
		
		if (DriverInputControl.getInstance().getPressureButton(
				RobotButtonType.TOTE_LIFT) > DEAD_ZONE) {

			this.isIncrementing = false;
			this.isDecrementing = false;
			this.isResetting = false;
			this.isSupported = false;
			this.reset();
			liftSpeed = DriverInputControl.getInstance().getPressureButton(
					RobotButtonType.TOTE_LIFT);
		} else if (DriverInputControl.getInstance().getPressureButton(
				RobotButtonType.TOTE_LIFT) < -DEAD_ZONE) {
			this.reset();
			this.isIncrementing = false;
			this.isDecrementing = false;
			this.isSupported = false;
			this.isResetting = false;

			liftSpeed = DriverInputControl.getInstance().getPressureButton(
					RobotButtonType.TOTE_LIFT);

		} else if (!isIncrementing && !isDecrementing && !isResetting) {
			liftSpeed = 0;
		}

		if (DriverInputControl.getInstance().getButton(RobotButtonType.LIFT_SUPPORT) && !prevSupportState) {
            this.liftSupported = !this.liftSupported;
    	}
		
		this.isSupported = this.liftSupported;
		
		prevSupportState = DriverInputControl.getInstance().getButton(RobotButtonType.LIFT_SUPPORT);
		
		updateLift(this.liftSpeed);
	}

	public void updateLift(double speed) {
		liftSpeed = speed;
		if (speed < 0) {
			state = MotorState.FORWARD;
		} else if (speed > 0) {
			state = MotorState.REVERSE;
		} else {
			state = MotorState.OFF;
		}

		this.desiredLiftSpeedTicks = this.liftSpeed * ToteLift.MAX_LIFT_SPEED_ENC;
		
		if (state == MotorState.FORWARD) {
			desiredLiftSpeedTicks = (desiredLiftSpeedTicks < ToteLift.MAX_LIFT_SPEED_UP_ENC ? ToteLift.MAX_LIFT_SPEED_UP_ENC
					: desiredLiftSpeedTicks);
			this.isSupported = false;

			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
				stop();
			}
		} else if (state == MotorState.REVERSE) {
			this.isSupported = false;
			desiredLiftSpeedTicks = (desiredLiftSpeedTicks > ToteLift.MAX_LIFT_SPEED_DOWN_ENC ? ToteLift.MAX_LIFT_SPEED_DOWN_ENC
					: desiredLiftSpeedTicks);
		} else {
			stop();
		}
		
		if (state == MotorState.OFF) {
			isBraked = true;
			this.isSupported = this.liftSupported;
			this.motorPowerControlLoop.reset();
		} else {
			isBraked = false;
			this.isSupported = false;
			
			System.out.println(SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER).getRate() + ", " + this.desiredLiftSpeedTicks);
			
			double currEncRate = -SensorInputControl.getInstance()
					.getEncoder(SensorType.TOTE_ENCODER).getRate();

			this.motorSpeed = -motorPowerControlLoop.getPID(this.desiredLiftSpeedTicks, -currEncRate);
			
		}
	}

	public boolean incrementLift(double distance, double error) {

        distanceTraveled = -SensorInputControl.getInstance()
                .getEncoder(SensorType.TOTE_ENCODER).getDistance() - relativeEncoder;

        
        double difference = Math.abs(distanceTraveled - distance);
        boolean isDone = difference <= error;
//        System.out
//                .println("incrementLift - [encoder distance, wanted distance, error, difference, isDone]"
//                        + distanceTraveled
//                        + ", "
//                        + distance
//                        + ", "
//                        + relativeEncoder
//                        + ", "
//                        + error
//                        + ", " + difference + ", " + isDone);

        if (isDone) {
            this.reset();
            this.stop();
            relativeEncoder += distanceTraveled;
            return true;
        } else {
            liftSpeed = distanceControlLoop.getPID(distance, distanceTraveled);
            updateLift(-liftSpeed);
//            System.out.println("incrementLift speed - " + liftSpeed);
            return false;
        }
    }

	public void reset() {
		distanceControlLoop.reset();
		this.motorPowerControlLoop.reset();
//		SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER)
//				.reset();
	}

	public void stop() {
		state = MotorState.OFF;
		liftSpeed = 0;
		desiredLiftSpeedTicks = 0;
		motorSpeed = 0;
	}

	public void updateOutputs() {
//		System.out.println(toString());
		//FIXME: do not commit, was -liftSpeed for non-testbed
		RobotControl.getInstance().updateToteMotor(motorSpeed);
		RobotControl.getInstance().updateToteStop(isBraked);
//		RobotControl.getInstance().updateToteSupport(isSupported);
	}

	@Override
	public void update() {
		// TODO Auto-generated method stub
		updateLift();
	}

	public double getMotorSpeed() {
		return motorSpeed;
	}

	public void setMotorSpeed(double motorSpeed) {
		this.motorSpeed = motorSpeed;
	}

	public void setScalingValue(double height) {
		this.distanceControlLoop.setScalingValue(height);
		
	}

}