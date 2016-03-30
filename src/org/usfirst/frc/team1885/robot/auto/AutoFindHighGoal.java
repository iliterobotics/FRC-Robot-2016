package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

public class AutoFindHighGoal extends AutoCommand{

    private final double DRIVE_SPEED = -0.2;// negative drive speed to back up until we can see the high goal
    
    private AutoDriveStart autoDriveStart;
    
    public AutoFindHighGoal(){
        autoDriveStart = new AutoDriveStart(DRIVE_SPEED);
    }
    
    @Override
    public boolean init() {
        autoDriveStart.init();
        return false;
    }

    @Override
    public boolean execute() {
        if(Shooter.getInstance().isGoalFound()){
            autoDriveStart.execute();
        }
        return Shooter.getInstance().isGoalFound();
    }

    @Override
    public boolean updateOutputs() {
        autoDriveStart.updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        autoDriveStart.reset();
    }

}
