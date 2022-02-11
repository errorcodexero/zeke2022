package frc.robot.gpm;

import org.xero1425.base.Subsystem;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zeke_color_sensor.ZekeColorSensor;

public class GPMSubsystem extends Subsystem {
  
    public static final String SubsystemName = "gamepiecemanipulator" ;

    private ZekeIntakeSubsystem intake_ ;
    private ConveyorSubsystem conveyor_ ;
    private ShooterSubsystem shooter_ ;
    private TurretSubsystem turret_ ;

    public GPMSubsystem(Subsystem parent, TankDriveSubsystem db, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName) ;

        intake_ = new ZekeIntakeSubsystem(this, color) ;
        addChild(intake_) ;

        conveyor_ = new ConveyorSubsystem(this, color) ;
        addChild(conveyor_) ;
        
        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_) ;

        turret_ = new TurretSubsystem(this) ;
        addChild(turret_) ;

    }

    public ZekeIntakeSubsystem getIntake() {
        return intake_ ;
    }

    public ConveyorSubsystem getConveyor() {
        return conveyor_ ;
    }

    public ShooterSubsystem getShooter() {
        return shooter_ ;
    }

    public TurretSubsystem getTurret() {
        return turret_ ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

}
