package frc.robot.automodes;

import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSetBall;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class FourBallAuto extends ZekeAutoMode {
    private final double FirstShotAngle = 30.0 ;

    public FourBallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TurretSubsystem turret = zeke.getTurret() ;

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Close the clamps so we don't violate height limits
        closeClamps();

        // Start the limelight
        startLimelightTracking() ;

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);
        driveAndCollect("fourball_p1", 0.0, 2.0, FirstShotAngle, null);

        // addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);

        // addSubActionPair(db, new TankDrivePathFollowerAction(db, "fourball_p2", true), true) ;
        // driveAndCollect("fourball_p3", 0.0, 0.0, 0.0, null);
        // driveAndFire("fourball_p4", true, 0.0) ;
    }
}
