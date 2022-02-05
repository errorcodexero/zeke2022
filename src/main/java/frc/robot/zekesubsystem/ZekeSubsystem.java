package frc.robot.zekesubsystem;

import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
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
    private GPMSubsystem gpm_ ;
    private ClimberSubsystem climber_ ;
    private ZekeColorSensor color_sensor_ ;

    public ZekeSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        db_ = new TankDriveSubsystem(this, TankdriveSubsystemName, "tankdrive") ;
        addChild(db_) ;

        oi_ = new ZekeOISubsystem(this, db_) ;
        addChild(oi_) ;

        color_sensor_ = new ZekeColorSensor(this, I2C.Port.kMXP) ;
        addChild(color_sensor_) ;

        gpm_ = new GPMSubsystem(this, db_, color_sensor_) ;
        addChild(gpm_) ;

        limelight_ = new ZekeLimeLightSubsystem(this) ;
        addChild(limelight_) ;

        turret_ = new TurretSubsystem(this) ;
        addChild(turret_) ;

        tracker_ = new TargetTrackerSubsystem(this, limelight_, turret_) ;
        addChild(tracker_) ;

        climber_ = new ClimberSubsystem(this) ;
        addChild(climber_) ;
    }

    public TurretSubsystem getTurret() {
        return turret_ ;
    }

    public TargetTrackerSubsystem getTargetTracker() {
        return tracker_ ;
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

    public GPMSubsystem getGPMSubsystem() {
        return gpm_ ;
    }
}
