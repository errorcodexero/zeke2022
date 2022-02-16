package frc.robot.automodes;

import java.lang.annotation.Target;

import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.ZekeIntakeArmAction;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.intake.ZekeIntakePowerAction;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.intake.ZekeIntakeThroughputAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekeoi.ZekeOISubsystem;
import frc.robot.climber.ClimbAction ;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeTestModeAuto extends TestAutoMode {
    ClimbAction caction_ = null ;

    public ZekeTestModeAuto(ZekeAutoController ctrl) throws Exception {
        super(ctrl, "Zeke2022-Test-Mode");

        ZekeSubsystem zeke = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        ZekeIntakeSubsystem intake = zeke.getGPMSubsystem().getIntake() ;   
        ConveyorSubsystem conveyor = zeke.getGPMSubsystem().getConveyor() ;
        ShooterSubsystem shooter = zeke.getGPMSubsystem().getShooter() ;
        TurretSubsystem turret = zeke.getTurret() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        ClimberSubsystem climber = zeke.getClimber() ;

        // https://docs.google.com/document/d/1_RRGDMPI7WE6HX6dVFBds36qcHX3i3mzn8hmVEGbd0s/edit
        // send an email if you can't access -> all the tests we'll need
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
            case 14:
                addSubActionPair(intake, new ZekeIntakeThroughputAction(intake), true);
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
            case 30: // power to both wheel motors; position to hood motor
                addSubActionPair(shooter, new SetShooterAction(shooter, getPower(), getPower(), getPosition()), false);
                break ;

            case 31: // power to wheel motor #1
                addSubActionPair(shooter, new SetShooterAction(shooter, getPower(), 0.0, 0.0), false);
                break ;
        
            case 32: // power to wheel motor #2
                addSubActionPair(shooter, new SetShooterAction(shooter, 0.0, getPower(), 0.0), false);
                break ;
                
            case 33: // power to both wheel motors
                addSubActionPair(shooter, new SetShooterAction(shooter, getPower(), getPower(), 0.0), false);
                break ;

            case 34: // position to hood motor
                addSubActionPair(shooter, new SetShooterAction(shooter, 0.0, 0.0, getPosition()), false);
                break ;

            //
            // Numbers 40 - 49 are for the LIMELIGHT/TURRET/TARGETTRACKER
            //
            case 40: // follow the limelight!
                addSubActionPair(turret, new FollowTargetAction(turret, tracker), false);
                break ;

            //
            // Numbers 50 - 59 are for the CLIMBER
            //
            case 50:        
                if (caction_ == null) {
                    caction_ = new ClimbAction(climber, db, zeke.getOI()) ;
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
