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

public class TwoBallLeftAuto extends ZekeAutoMode {

    private final double FirstFireAngle = 0.0 ;
    private final double ShooterWheelsSpinupSpeed = 4000.0 ;
    private final double FirstFireHood = 6.0 ;

    public TwoBallLeftAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TurretSubsystem turret = zeke.getTurret() ;
        ShooterSubsystem shooter = gpm.getShooter() ;
        
        // Set state of the conveyor to reflect a single ball preloaded
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);

        // Close the clamps so we don't violate height limits
        closeClamps();        

        // Start the shooter wheels so they are ready to fire
        var shoot = new SetShooterAction(shooter, ShooterWheelsSpinupSpeed, ShooterWheelsSpinupSpeed, FirstFireHood) ;

        // Drive and collect the second ball
        driveAndCollect("twoball_p1", 1.0, 0.0, FirstFireAngle, shoot);

        // Start the limelight
        startLimelightTracking();

        // Start firing the two balls
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
