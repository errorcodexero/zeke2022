package frc.robot.automodes;

import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSetBall;
import frc.robot.gpm.GPMFakeFire;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class NearTarmac4BallAuto extends ZekeAutoMode {
    public NearTarmac4BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TurretSubsystem turret = zeke.getTurret() ;

        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);
        driveAndCollect("near_tarmac_4_p1", 0.0, 2.0);

        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_4_p2", true), true) ;
        driveAndCollect("near_tarmac_4_p3", 0.0, 0.0);

        driveAndFire("near_tarmac_4_p4", true, 0.0) ;
    }
}
