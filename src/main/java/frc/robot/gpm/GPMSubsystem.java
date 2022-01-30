package frc.robot.gpm;

import org.xero1425.base.Subsystem;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.shooter.ShooterSubsystem;

public class GPMSubsystem extends Subsystem {
  
    public static final String SubsystemName = "gamepiecemanipulator" ;

    // add the conveyor and shooter as they get created...
    private ZekeIntakeSubsystem intake_ ;
    private ShooterSubsystem shooter_ ;

    public GPMSubsystem(Subsystem parent, TankDriveSubsystem db) throws Exception {
        super(parent, SubsystemName) ;

        intake_ = new ZekeIntakeSubsystem(this) ;
        addChild(intake_) ;
        
        shooter_ = new ShooterSubsystem(this);
        addChild(shooter_);
    }

    public ZekeIntakeSubsystem getIntake() {
        return intake_ ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

}
