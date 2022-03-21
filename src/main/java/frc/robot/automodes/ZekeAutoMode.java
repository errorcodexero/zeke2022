package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.climber.ChangeClampAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMStopCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.shooter.SetShooterAction;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeAutoMode extends AutoMode {
    public ZekeAutoMode(ZekeAutoController ctrl, String name) {
        super(ctrl, name);
    }

    protected ZekeSubsystem getZekeRobotSubsystem() {
        return (ZekeSubsystem) getAutoController().getRobot().getRobotSubsystem();
    }

    protected void closeClamps() throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        ZekeSubsystem zeke = getZekeRobotSubsystem() ;
        ClimberSubsystem climber = zeke.getClimber() ;

        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.CLOSED), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.CLOSED), false);
    }

    protected void startLimelightTracking() throws BadParameterTypeException, MissingParameterException, InvalidActionRequest {
        ZekeSubsystem zeke = getZekeRobotSubsystem() ;
        FollowTargetAction act = new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker()) ;
        addSubActionPair(zeke.getTurret(), act, false);
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
        TurretSubsystem turret = zeke.getTurret() ;
        ParallelAction parallel;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);
        parallel.addSubActionPair(turret, new MotorEncoderGotoAction(turret, angle, true), false) ;
        parallel.addSubActionPair(zeke.getTankDrive(), new TankDrivePathFollowerAction(zeke.getTankDrive(), path, reverse), true);
        addAction(parallel);
        
        startLimelightTracking();
        addSubActionPair(gpm, new GPMFireAction(gpm, zeke.getTargetTracker(), zeke.getTankDrive(), zeke.getTurret(), false), true) ;
    }

        //
    // Add a sequence that drives a path and fires the balls in the robot once the path is
    // complete.
    //
    // If the path is null, then no driving is performed before firing the balls.
    //
    protected void driveAndFireMoving(String path, boolean reverse, double angle) throws Exception {
        
        ZekeSubsystem zeke = getZekeRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem();
        TurretSubsystem turret = zeke.getTurret() ;
        ParallelAction parallel;
        SequenceAction series ;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);

        // This series points the turret in the right direction, starts limelight tracking, and then fires on the move
        series = new SequenceAction(getAutoController().getRobot().getMessageLogger()) ;
        series.addSubActionPair(turret, new MotorEncoderGotoAction(turret, angle, true), false) ;
        FollowTargetAction act = new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker()) ;
        series.addSubActionPair(zeke.getTurret(), act, false);
        series.addSubActionPair(gpm, new GPMFireAction(gpm, zeke.getTargetTracker(), zeke.getTankDrive(), zeke.getTurret(), true), true) ;
        parallel.addAction(series);

        // This action puts the path in parallel with the firing related activities
        parallel.addSubActionPair(zeke.getTankDrive(), new TankDrivePathFollowerAction(zeke.getTankDrive(), path, reverse), true);

        addAction(parallel);
    }

    //
    // Add a sequence that drives a path and collects balls along the path.  This assumes that the
    // collection sequence can be executed along the start of the path so that no delay is necessary
    // before following the path to let the collection sequence start.
    //
    protected void driveAndCollect(String path, double delay1, double delay2, double angle, SetShooterAction act) throws Exception {
        GPMSubsystem gpm = getZekeRobotSubsystem().getGPMSubsystem();
        TankDriveSubsystem db = getZekeRobotSubsystem().getTankDrive();
        TurretSubsystem turret = getZekeRobotSubsystem().getTurret() ;
        ParallelAction parallel;
        SequenceAction drive;

        parallel = new ParallelAction(getAutoController().getRobot().getMessageLogger(), ParallelAction.DonePolicy.All);

        //
        // The drive series that delays, drives, then delays.  This series should controle the life time of the
        // parallel.
        //

        drive = new SequenceAction(getAutoController().getRobot().getMessageLogger());

        if (Math.abs(delay1) > 0.05) {
            drive.addAction(new DelayAction(getAutoController().getRobot(), delay1));
        }
        drive.addSubActionPair(db, new TankDrivePathFollowerAction(db, path, false), true);
        if (Math.abs(delay2) > 0.05) {
            drive.addAction(new DelayAction(getAutoController().getRobot(), delay2));
        } 
        parallel.addAction(drive);

        //
        // The turret positioning so we are sure the turret is pointed at the lime
        // light when we are done.
        //
        parallel.addSubActionPair(turret, new MotorEncoderGotoAction(turret, angle, true), true);

        //
        // The collect sequence
        //
        GPMStartCollectAction collect = new GPMStartCollectAction(gpm, act) ;
        parallel.addSubActionPair(gpm, collect, false) ;

        //
        // Now add the parallel to the action to the automode that does the drive and collect with
        // turret alignment at the end of the drive
        //
        addAction(parallel);

        //
        // When the path and delay is done, stop collecting
        //
        GPMStopCollectAction stop = new GPMStopCollectAction(gpm, act) ;
        addSubActionPair(gpm, stop, false);
    }
}
