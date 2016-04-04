
package org.usfirst.frc.team1885.robot.config2016;

import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMap;
import org.usfirst.frc.team1885.robot.common.type.JoystickButtonMatch;
import org.usfirst.frc.team1885.robot.common.type.ModuleType;
import org.usfirst.frc.team1885.robot.common.type.RelayType;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ModuleControl;
import org.usfirst.frc.team1885.robot.modules.Shooter;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * 
 * @author ILITE Robotics
 * @version This class keeps track of all joy stick mappings and sensor-to-talon
 *          mappings for the robot Also keeps track of ports for motors
 */
public class RobotConfiguration {
    public static final double WHEEL_DIAMETER = 8.914666748046875; // magic,
                                                                   // needs to
                                                                   // be
                                                                   // configured
                                                                   // every time
    public static final int ARM_JOINT_A_PORT = 1; // 5 on real robot
    public static final int ARM_JOINT_B_PORT = 2; // 6 on real robot
    public static final double MAX_SPEED = .8;

    /**
     * Method adds all joy stick mappings and sensor-to-talon mappings
     */
    public static void configureRobot() {
        RobotControlWithSRX robotControl = RobotControlWithSRX.getInstance();
        DriverInputControlSRX driverInputControl = DriverInputControlSRX.getInstance();
        SensorInputControlSRX sensorInputControl = SensorInputControlSRX.getInstance();
        JoystickButtonMap joystickButtonMap = JoystickButtonMap.getInstance();
        ModuleControl moduleControl = ModuleControl.getInstance();

        // Add JoystickType Configurations
        driverInputControl.addJoystick(RobotJoystickType.LEFT_DRIVE, new Joystick(0));
        driverInputControl.addJoystick(RobotJoystickType.RIGHT_DRIVE, new Joystick(1));
        driverInputControl.addJoystick(RobotJoystickType.CONTROLLER, new Joystick(2));
        driverInputControl.addJoystick(RobotJoystickType.OVERRIDE_CONTROLLER, new Joystick(3));

        // Add Joystick Button Mappings
        // Drivetrain Control Joystick Mappings
        joystickButtonMap.addControllerButton(RobotButtonType.GEAR_SHIFT, new JoystickButtonMatch(RobotJoystickType.LEFT_DRIVE, 3));
        // Shooter Joystick Mappings
        joystickButtonMap.addControllerButton(RobotButtonType.FLYWHEEL_IN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 5));
        joystickButtonMap.addControllerButton(RobotButtonType.FLYWHEEL_INTAKE_IN, new JoystickButtonMatch(RobotJoystickType.RIGHT_DRIVE, 2));
        joystickButtonMap.addControllerButton(RobotButtonType.FLYWHEEL_OUT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 7));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_LAUNCH, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 8));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_TILT_UP, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 4));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_TILT_DOWN, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 2));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_TILT, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, Joystick.AxisType.kY));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_TWIST_LEFT, new JoystickButtonMatch(RobotJoystickType.OVERRIDE_CONTROLLER, 1));
        joystickButtonMap.addControllerButton(RobotButtonType.SHOOTER_TWIST_RIGHT, new JoystickButtonMatch(RobotJoystickType.OVERRIDE_CONTROLLER, 3));
        joystickButtonMap.addControllerButton(RobotButtonType.FIRE, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 6));
        joystickButtonMap.addControllerButton(RobotButtonType.TACTICAL_LIGHT, new JoystickButtonMatch(RobotJoystickType.RIGHT_DRIVE, 1));
        // Active Intake Joystick Mappings
        joystickButtonMap.addControllerButton(RobotButtonType.INTAKE_SOLENOID, new JoystickButtonMatch(RobotJoystickType.LEFT_DRIVE, 1));
        joystickButtonMap.addControllerButton(RobotButtonType.INTAKE_IN, new JoystickButtonMatch(RobotJoystickType.RIGHT_DRIVE, 2));
        joystickButtonMap.addControllerButton(RobotButtonType.INTAKE_OUT, new JoystickButtonMatch(RobotJoystickType.RIGHT_DRIVE, 3));
        joystickButtonMap.addControllerButton(RobotButtonType.CDF_DROPPER, new JoystickButtonMatch(RobotJoystickType.LEFT_DRIVE, 2));
        // Arm Joystick Mappings
        joystickButtonMap.addControllerButton(RobotButtonType.ARM_MOVE_Y, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, Joystick.AxisType.kTwist));
        joystickButtonMap.addControllerButton(RobotButtonType.ARM_MOVE_X, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, Joystick.AxisType.kThrottle));
        joystickButtonMap.addControllerButton(RobotButtonType.RESET_BUTTON, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 9));
        joystickButtonMap.addControllerButton(RobotButtonType.AIM, new JoystickButtonMatch(RobotJoystickType.CONTROLLER, 0));

        // Talon Outputs for Robot
        robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
        robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
        robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 3);
        robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 4);
        robotControl.addTalonOutput(RobotMotorType.ARM_JOINT_A, 6);
        robotControl.addTalonOutput(RobotMotorType.ARM_JOINT_B, 5);
        robotControl.addTalonOutput(RobotMotorType.ACTIVE_INTAKE, 7);
        robotControl.addTalonOutput(RobotMotorType.FLYWHEEL_LEFT, 8);
        robotControl.addTalonOutput(RobotMotorType.FLYWHEEL_RIGHT, 9);
        robotControl.addTalonOutput(RobotMotorType.SHOOTER_TILT, 10);
        robotControl.addTalonOutput(RobotMotorType.SHOOTER_TWIST, 11);

        // Solenoids and DoubleSolenoids
        robotControl.addDoubleSolenoid(RobotPneumaticType.INTAKE_SETTER, 0);
        robotControl.addSingleSolenoid(RobotPneumaticType.GEAR_SHIFT, 2);
        robotControl.addSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER, 4);
        robotControl.addSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER, 3);
        robotControl.addSingleSolenoid(RobotPneumaticType.CDF_DROPPER, 5);

        // Relays
        robotControl.addRelay(RelayType.TACTICAL_LIGHT, 0);

        // Sensors
        sensorInputControl.createNavX(SerialPort.Port.kMXP);
        sensorInputControl.addPressureSensor(0);
        sensorInputControl.addRotarySwitchSensor(SensorType.DEFENSE_SELECTION, 1);
//        sensorInputControl.addRotarySwitchSensor(SensorType.TYPE_2, 2);
//        sensorInputControl.addRotarySwitchSensor(SensorType.TYPE_3, 3);
//        sensorInputControl.addRotarySwitchSensor(SensorType.TYPE_4, 4);
//        sensorInputControl.addBeamSensor(2);
        // Drivetrain Encoders
        robotControl.addTalonSensor(RobotMotorType.LEFT_DRIVE, SensorType.LEFT_ENCODER, 1);
        robotControl.addTalonSensor(RobotMotorType.RIGHT_DRIVE, SensorType.RIGHT_ENCODER, 2);
        // Shooter Encoders and Potentiometer
        robotControl.addTalonSensor(RobotMotorType.FLYWHEEL_LEFT, SensorType.FLYWHEEL_LEFT_ENCODER, 8);
        robotControl.addTalonSensor(RobotMotorType.FLYWHEEL_RIGHT, SensorType.FLYWHEEL_RIGHT_ENCODER, 9);
        robotControl.addTalonSensor(RobotMotorType.SHOOTER_TILT, SensorType.SHOOTER_TILT_POTENTIOMETER, 10);
        robotControl.addTalonSensor(RobotMotorType.SHOOTER_TWIST, SensorType.SHOOTER_TWIST_ENCODER, 11);
        // Utility Arm potentiometers
        RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.ARM_JOINT_A, SensorType.JOINT_A_CTRE_ABSOLUTE, 1);
        RobotControlWithSRX.getInstance().addTalonSensor(RobotMotorType.ARM_JOINT_B, SensorType.JOINT_B_CTRE_ABSOLUTE, 2);

        // Add Module
//        moduleControl.addModule(ModuleType.DRIVE_TRAIN, DrivetrainControl.getInstance());
//        moduleControl.addModule(ModuleType.ACTIVE_INTAKE, ActiveIntake.getInstance());
        moduleControl.addModule(ModuleType.SHOOTER, Shooter.getInstance());
    }
}