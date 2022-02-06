package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.LambdaAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMStopCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.shooter.ShooterSubsystem;
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
        
        GPMSubsystem gpm = getZekeRobotSubsystem().getGPMSubsystem();
        TankDriveSubsystem db = getZekeRobotSubsystem().getTankDrive();
        ShooterSubsystem shooter = getZekeRobotSubsystem().getGPMSubsystem().getShooter();
        ParallelAction parallel;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);
        parallel.addAction(setTurretToTrack(angle));

        if (path != null) {
            parallel.addSubActionPair(db, new TankDrivePathFollowerAction(db,path,reverse), true);
        }

        //TODO: add shooter action
        //parallel.addSubActionPair(shooter, new , block);

        addAction(parallel);
        addSubActionPair(gpm, new GPMFireAction(gpm), true);
    }

    //
    // Add a sequence that drives a path and collects balls along the path.  This assumes that the
    // collection sequence can be executed along the start of the path so that no delay is necessary
    // before following the path to let the collection sequence start.
    //
    protected void driveAndCollect(String path, double delay1, double delay2) throws Exception {
        GPMSubsystem gpm = getZekeRobotSubsystem().getGPMSubsystem();
        TankDriveSubsystem db = getZekeRobotSubsystem().getTankDrive();
        ParallelAction parallel;
        SequenceAction series, series2;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);

        series2 = new SequenceAction(getAutoController().getRobot().getMessageLogger());
        series2.addAction(new DelayAction(getAutoController().getRobot(), delay1));
        series2.addSubActionPair(db, new TankDrivePathFollowerAction(db, path, false), true);
        series2.addAction(new DelayAction(getAutoController().getRobot(), delay2));
        parallel.addAction(series2);

        GPMStartCollectAction collect = new GPMStartCollectAction(gpm) ;
        series = new SequenceAction(getAutoController().getRobot().getMessageLogger()); 
        series.addSubActionPair(gpm, collect, false);
        parallel.addAction(series);

        addAction(parallel);

        addAction(new LambdaAction(getAutoController().getRobot().getMessageLogger(), "FinishCollect", ()-> collect.finish())) ;
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
