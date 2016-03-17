package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.ActiveIntake;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoPortcullis extends AutoCommand {

    private enum Stage{
        FIRST/*Initial defense reach*/, SECOND/*Move until flat on defense*/, THIRD/*move until flat on ground*/, FINISHED
    }
    private Stage stage;
    private AutoDriveStart drive;
    private AutoCrossedDefense flat;
    private AutoReachedDefense tiltDown;
    
    public AutoPortcullis(){
        drive = new AutoDriveStart(0.3);
        flat = new AutoCrossedDefense();
        tiltDown = new AutoReachedDefense();
        stage = Stage.FIRST;
    }
    
    @Override
    public boolean init() {
        RobotControlWithSRX.getInstance().updateIntakeMotor(ActiveIntake.INTAKE_SPEED);        
        return false;
    }

    @Override
    public boolean execute() {
        ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
        drive.execute();
        switch(stage){
            case FIRST: if(flat.execute()){stage = Stage.SECOND;}break;
            case SECOND: if(tiltDown.execute()){stage = Stage.THIRD; flat.init();}break;
            case THIRD: if(flat.execute()){stage = Stage.FINISHED;}break;
            default: return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        ActiveIntake.getInstance().update();
        ActiveIntake.getInstance().updateOutputs();
        drive.updateOutputs();
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
