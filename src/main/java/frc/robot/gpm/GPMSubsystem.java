package frc.robot.gpm;

import org.xero1425.base.Subsystem;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.zeke_color_sensor.ZekeColorSensor;

public class GPMSubsystem extends Subsystem {
  
    // Butch: just for consistency, I renamed this to GPM like the actual class name
    public static final String SubsystemName = "gpm" ;

    private ZekeIntakeSubsystem intake_ ;
    private ConveyorSubsystem conveyor_ ;
    private ShooterSubsystem shooter_ ;

    public GPMSubsystem(Subsystem parent, TankDriveSubsystem db, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName) ;

        intake_ = new ZekeIntakeSubsystem(this, color) ;
        addChild(intake_) ;

        conveyor_ = new ConveyorSubsystem(this, color) ;
        addChild(conveyor_) ;
        
        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_) ;

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

    @Override
    public void run() throws Exception {
        super.run() ;
    }

}
