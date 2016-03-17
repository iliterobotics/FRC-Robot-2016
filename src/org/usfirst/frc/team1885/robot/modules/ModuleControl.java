package org.usfirst.frc.team1885.robot.modules;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.ModuleType;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.modules.drivetrain.DrivetrainControl;

public class ModuleControl{
	private static ModuleControl instance = null;
	private HashMap<ModuleType, Module> modules;
	
	protected ModuleControl() {
		modules = new HashMap<ModuleType, Module>();
	}
	
	public static ModuleControl getInstance() {
		if (instance == null) {
			instance = new ModuleControl();
		}
		return instance;
	}
	
	public Module[] getModules() {
		return this.modules.values().toArray(new Module[modules.size()]);
	}
	
	public void addModule(ModuleType module_type, Module module) {
		modules.put(module_type, module);
	}
	
	public Module get(ModuleType module_type) {
		return modules.get(module_type);
	}
	
	public DrivetrainControl getDriveTrain() {
		return (DrivetrainControl)modules.get(ModuleType.DRIVE_TRAIN);
	}
	
	public ActiveIntake getActiveIntake(){
	    return (ActiveIntake)modules.get(ModuleType.ACTIVE_INTAKE);
	}
	
	public Shooter getShooter() {
        return (Shooter)modules.get(ModuleType.SHOOTER);
    }
	
	public UtilityArm getUtilitArm(){
	    return (UtilityArm)modules.get(ModuleType.UTILITY_ARM);
	}
	
	public void updateIntakeShooter(){
	    Shooter.getInstance().update();
	    ActiveIntake.getInstance().update();
	}
	
	public void updateIntakeShooterOutputs(){
	    Shooter.getInstance().updateOutputs();
        ActiveIntake.getInstance().updateOutputs();
	}
}
