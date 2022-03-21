package frc.robot.automodes;

import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSetBall;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.shooter.SetShooterAction;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ThreeBallAuto extends ZekeAutoMode {

    private final double FirstFireAngle = 30.0 ;
    private final double SecondFireAngle = 30.0 ;
    private final double FirstFireHood = 20.0 ;
    private final double ShooterWheelsSpinupSpeed = 6000.0 ;

    public ThreeBallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TurretSubsystem turret = zeke.getTurret() ;
        ShooterSubsystem shooter = gpm.getShooter();

        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Close the clamps so we don't violate height limits
        closeClamps();

        // Start the shooter wheels so they are ready to fire
        var shoot = new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, ShooterWheelsSpinupSpeed, FirstFireHood) ;

        // drive and collect the second ball
        driveAndCollect("threeball_p1", 0.5, 0.0, FirstFireAngle, shoot) ;

        // Start the limelight
        startLimelightTracking() ;

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret, true), true);

        // Drive and collect the ball near the terminal
        driveAndCollect("threeball_p2", 0.0, 0.5, 0.0, shoot);

        // Start the shooter wheels so they are ready to fire
        addSubActionPair(shooter, new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, ShooterWheelsSpinupSpeed, FirstFireHood), false) ;

        // Drive back to the target and fire the ball
        driveAndFire("threeball_p3", true, SecondFireAngle) ;
    }
}
