package frc.robot.automodes;

import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.controllers.AutoMode;

import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeAutoMode extends AutoMode {
    public ZekeAutoMode(ZekeAutoController ctrl, String name) {
        super(ctrl, name);
    }

    protected ZekeSubsystem getZekeRobotSubsystem() {
        return (ZekeSubsystem) getAutoController().getRobot().getRobotSubsystem();
    }
    
    //
    // Add the actions to set the initial ball count
    //
    protected void setInitialBallCount(int count) {
        //
    }

    //
    // Initialize the climber
    //
    protected void initializeClimber() throws Exception {
        //
    }

    //
    // Add a sequence that drives a path and fires the balls in the robot once the path is
    // complete.
    //
    // If the path is null, then no driving is performed before firing the balls.
    //
    protected void driveAndFire(String path, boolean reverse, double angle) throws Exception {
        // TankDriveSubsystem db = getZekeRobotSubsystem().getTankDrive() ;
      
    }

    //
    // Add a sequence that drives a path and collects balls along the path.  This assumes that the
    // collection sequence can be executed along the start of the path so that no delay is necessary
    // before following the path to let the collection sequence start.
    //
    protected void driveAndCollect(String path, double delay1, double delay2) throws Exception {
        // TankDriveSubsystem db = getZekeRobotSubsystem().getTankDrive() ;
    }

    //
    // Add a sequence to move the turret to a specific angle and then set it up to track the
    // target.
    //
    protected SequenceAction setTurretToTrack(double angle) throws Exception {
        SequenceAction seq = new SequenceAction(getAutoController().getRobot().getMessageLogger()) ;
        // still need to write stuff here :)
        return seq ;
    }

}
