package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.modules.Shooter;

/**
 * This class will control the movement to center to the high goal once the goal can be found
 * 1) Calculate the x offset from the goal
 * 2) Turn +-90 and move the distance to center the robot (involves AutoDriveDistance)
 * 3) turn back to the goal and adjust tilt for y offset (involves AutoAimTurn)
 * @author ILITE Robotics
 *
 */
public class AutoLineUpHighGoal extends AutoCommand{

    private final double TURN = 90.0;
    
    private Stage stage;
    
    private AutoAlign autoAlign;
    private AutoDriveDistance autoDriveDistance;
    private AutoAimTurn autoAimTurn;
    private double distance;
    
    private double sideFactor; //positive assumes that we are to the left of the goal, we will turn a positive degree(to the right)
    
    public AutoLineUpHighGoal(){
        stage = Stage.INIT_TURN;
        distance = 0;
        sideFactor = 1;
    }
    
    @Override
    public boolean init() {        
        double straightDistance = Shooter.getInstance().getDistanceToGoal();
        double angleOffset = Shooter.getInstance().getTwistAimLock();
        distance = straightDistance * Math.cos(angleOffset);
        
        sideFactor = angleOffset > 0 ? 1 : -1;
        
        autoAlign = new AutoAlign(TURN * sideFactor);
        autoDriveDistance = new AutoDriveDistance(distance);
        autoAimTurn = new AutoAimTurn(TURN * -sideFactor);
        
        autoAlign.init();
        autoDriveDistance.init();
        autoAimTurn.init();
        return false;
    }

    @Override
    public boolean execute() {
        switch(stage){
            case INIT_TURN: autoAlign.execute(); break;
            case DRIVE: autoDriveDistance.execute(); break;
            case FINAL_TURN: autoAimTurn.execute(); break;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        switch(stage){
            case INIT_TURN: autoAlign.updateOutputs(); break;
            case DRIVE: autoDriveDistance.updateOutputs(); break;
            case FINAL_TURN: autoAimTurn.updateOutputs(); break;
        }
        return false;
    }

    @Override
    public void reset() {
    }
    
    private enum Stage {
        INIT_TURN, DRIVE, FINAL_TURN
    }

}
