package frc.robot.automodes;

import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeTestModeAuto extends TestAutoMode {
    public ZekeTestModeAuto(ZekeAutoController ctrl) throws Exception {
        super(ctrl, "Zeke2022-Test-Mode");

        ZekeSubsystem zeke = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem();
        TankDriveSubsystem db = zeke.getTankDrive();

        switch (getTestNumber()) {
            //
            // Numbers 0 - 9 are for the driverbase
            //
            case 0: // Drive straight, used to test and get Kv number
                addSubActionPair(db, new TankDrivePowerAction(db, getPower(), getPower(), getDuration()), true);
                break;

            case 1:
                addSubActionPair(db, new TankDrivePathFollowerAction(db, getNameParam(), false), true) ;
                break ;  
                        
            //
            // Numbers 10 - 19 are for the intake
            //
            case 10:        
                //add action
                break ;
            
            //
            // Numbers 20 - 29 are for the conveyor
            //
            case 20:        
                //add action
                break ;
            
            //
            // Numbers 30 - 39 are for the shooter
            //
            case 30:        
                //add action
                break ;

            //
            // Numbers 40 - 49 are for the turret/limelight
            //
            case 40:        
                //add action
                break ;

            //
            // Numbers 50 - 59 are for the climber
            //
            case 50:        
                //add action
                break ;

            //
            // Numbers 100+ are for the whole-robot
            //
            case 100:        
                //add action
                break ;
        }
    }
}