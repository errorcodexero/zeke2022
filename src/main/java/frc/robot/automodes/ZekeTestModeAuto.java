package frc.robot.automodes;

import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.ZekeIntakeArmAction;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.intake.ZekeIntakePowerAction;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.climber.ClimbAction ;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeTestModeAuto extends TestAutoMode {
    ClimbAction caction_ = null ;

    public ZekeTestModeAuto(ZekeAutoController ctrl) throws Exception {
        super(ctrl, "Zeke2022-Test-Mode");

        ZekeSubsystem zeke = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem();
        TankDriveSubsystem db = zeke.getTankDrive();
        ClimberSubsystem climber = zeke.getClimber() ;
        ZekeIntakeSubsystem intake = zeke.getGPMSubsystem().getIntake() ;

        switch (getTestNumber()) {
            //
            // Numbers 0 - 9 are for the DRIVEBASE
            //
            case 0: // Drive straight, used to test and get Kv number
                addSubActionPair(db, new TankDrivePowerAction(db, getPower(), getPower(), getDuration()), true);
                break;

            case 1:
                // addSubActionPair(db, new TankDrivePathFollowerAction(db, getNameParam(), false), true) ;
                break ;  
                        
            //
            // Numbers 10 - 19 are for the INTAKE
            //
            case 10:        
                addSubActionPair(intake, new ZekeIntakeOnAction(intake), true) ;
                break ;

            case 11:  // turns on the left intake motor
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, ZekeIntakePowerAction.IntakeMotor.LEFT, getPower()), true);            
                break ;
            
            case 12:  // turns on the right intake motor
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, ZekeIntakePowerAction.IntakeMotor.RIGHT, getPower()), true);            
                break ;

            case 13:  // retracts and deploys the intake arm
                // add the waits into place between deploying and retracting
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.DEPLOY), false);  
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.RETRACT), false);  
                break ;

            //
            // Numbers 20 - 29 are for the CONVEYOR
            //
            case 20:        
                //add action
                break ;
            
            //
            // Numbers 30 - 39 are for the SHOOTER
            //
            case 30:        
                //add action
                break ;

            //
            // Numbers 40 - 49 are for the LIMELIGHT/TURRET
            //
            case 40:        
                //add action
                break ;

            //
            // Numbers 50 - 59 are for the CLIMBER
            //
            case 50:        
                if (caction_ == null) {
                    caction_ = new ClimbAction(climber, db) ;
                }
                addSubActionPair(climber, caction_, true);      
                break ;

            //
            // Numbers 100+ are for the whole-robot; gpm; etc
            //
            case 100:  
                break ;

        }
    }
}
