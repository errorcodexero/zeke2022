package frc.robot.automodes;

import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSetBall;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class FarTarmac4BallAuto extends ZekeAutoMode {

    public FarTarmac4BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TurretSubsystem turret = zeke.getTurret() ;

        startTracking() ;
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);
        driveAndCollect("far_tarmac_4_ball_1", 0.5, 0.0);
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        driveAndCollect("far_tarmac_4_ball_2", 0.0, 0.0);

        addSubActionPair(zeke.getTurret(), new MotorEncoderGotoAction(zeke.getTurret(), 30.0, true), false);
        addSubActionPair(db, new TankDrivePathFollowerAction(db, "far_tarmac_4_ball_3", true), true) ;
        startTracking();

        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
