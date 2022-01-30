package frc.robot.zekesubsystem; //directory path from "java"

import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.climber.ClimberSubsystem;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekeoi.ZekeOISubsystem;
import frc.robot.zeke_limelight.ZekeLimeLightSubsystem;

public class ZekeSubsystem extends RobotSubsystem {
    public final static String SubsystemName = "zeke" ;
    public final static String TankdriveSubsystemName = "tankdrive" ;
    private TankDriveSubsystem db_ ;
    private ZekeOISubsystem oi_;
    private ZekeLimeLightSubsystem limelight_ ;
    private TurretSubsystem turret_ ;
    private TargetTrackerSubsystem tracker_ ;
    private ShooterSubsystem shooter_;
    
    private GPMSubsystem gpm_ ;
    private ClimberSubsystem climber_ ;
    private ZekeIntakeSubsystem intake_ ;


    public ZekeSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        db_ = new TankDriveSubsystem(this, TankdriveSubsystemName, "tankdrive") ;
        addChild(db_) ;

        oi_ = new ZekeOISubsystem(this, db_) ;
        addChild(oi_) ;

        gpm_ = new GPMSubsystem(this, db_) ;
        addChild(gpm_) ;

        limelight_ = new ZekeLimeLightSubsystem(this) ;
        addChild(limelight_) ;

        turret_ = new TurretSubsystem(this) ;
        addChild(turret_) ;

        tracker_ = new TargetTrackerSubsystem(this, limelight_, turret_) ;
        addChild(tracker_) ;

        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_);

        climber_ = new ClimberSubsystem(this) ;
        addChild(climber_) ;

        intake_ = new ZekeIntakeSubsystem(this) ;
        addChild(intake_) ;

    }

    public TankDriveSubsystem getTankDrive() {
        return db_ ;
    }

    public ZekeOISubsystem getOI() {
        return oi_ ;
    }

    public ClimberSubsystem getClimber() {
        return climber_ ;
    }
}