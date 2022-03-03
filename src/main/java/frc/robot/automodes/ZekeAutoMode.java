package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.LambdaAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMStopCollectAction;
import frc.robot.gpm.GPMSubsystem;
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
    protected void setInitialBallCount(int count) throws InvalidActionRequest {
        ConveyorSubsystem conveyor = getZekeRobotSubsystem().getGPMSubsystem().getConveyor() ;
        LambdaAction a = new LambdaAction(getAutoController().getRobot().getMessageLogger(), "set-ball-count",  () -> { conveyor.setPreloadedBall(); }) ;
        addAction(a);
    }

    //
    // Add a sequence that drives a path and fires the balls in the robot once the path is
    // complete.
    //
    // If the path is null, then no driving is performed before firing the balls.
    //
    protected void driveAndFire(String path, boolean reverse, double angle) throws Exception {
        
        ZekeSubsystem zeke = getZekeRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem();
        ParallelAction parallel;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);
        parallel.addAction(setTurretToTrack(angle));

        if (path != null) {
            parallel.addSubActionPair(zeke.getTankDrive(), new TankDrivePathFollowerAction(zeke.getTankDrive(), path, reverse), true);
        }
        addAction(parallel);
        addSubActionPair(gpm, new GPMFireAction(gpm, zeke.getTargetTracker(), zeke.getTankDrive(), zeke.getTurret()), true) ;
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

        if (Math.abs(delay1) > 0.05) {
            series2.addAction(new DelayAction(getAutoController().getRobot(), delay1));
        }
        series2.addSubActionPair(db, new TankDrivePathFollowerAction(db, path, false), true);
        if (Math.abs(delay2) > 0.05) {
            series2.addAction(new DelayAction(getAutoController().getRobot(), delay2));
        } 
        parallel.addAction(series2);

        GPMStartCollectAction collect = new GPMStartCollectAction(gpm) ;
        series = new SequenceAction(getAutoController().getRobot().getMessageLogger()); 

        //
        // Note this collect sequence is a non-blocking dispatch.  The dispatch action will
        // complete right away and so we really wait for the path action to complete.
        //
        series.addSubActionPair(gpm, collect, false);
        parallel.addAction(series);
        addAction(parallel);

        addSubActionPair(gpm, new GPMStopCollectAction(gpm), true);
    }

    //
    // Add a sequence to move the turret to a specific angle and then set it up to track the
    // target.
    //
    protected SequenceAction setTurretToTrack(double angle) throws Exception {
        SequenceAction seq = new SequenceAction(getAutoController().getRobot().getMessageLogger()) ;
        MotorEncoderGotoAction action = new MotorEncoderGotoAction(getZekeRobotSubsystem().getTurret(), angle, true) ;
        seq.addSubActionPair(getZekeRobotSubsystem().getTurret(), action, true);
        return seq ;
    }
}
