package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.LambdaAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.climber.ClimberSubsystem;
import frc.robot.conveyor.ConveyorExitAction;
import frc.robot.conveyor.ConveyorPowerAction;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.intake.ZekeIntakeArmAction;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.intake.ZekeIntakePowerAction;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.shooter.SetShooterAction;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.climber.ClimbAction ;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeTestModeAuto extends TestAutoMode {
    ClimbAction caction_ = null ;
    LambdaAction a1_ ;
    LambdaAction a2_ ;

    public ZekeTestModeAuto(ZekeAutoController ctrl) throws Exception {
        super(ctrl, "Zeke2022-Test-Mode");

        MessageLogger logger = ctrl.getRobot().getMessageLogger() ;
        ZekeSubsystem zeke = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        ZekeIntakeSubsystem intake = zeke.getGPMSubsystem().getIntake() ;   
        ConveyorSubsystem conveyor = zeke.getGPMSubsystem().getConveyor() ;
        ShooterSubsystem shooter = zeke.getGPMSubsystem().getShooter() ;
        TurretSubsystem turret = zeke.getTurret() ;
        // TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        // ClimberSubsystem climber = zeke.getClimber() ;

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
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, getPower(), 0.0, getDuration()), true);            
                break ;
            
            case 12:  // turns on the right intake motor
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, 0.0, getPower(), getDuration()), true);            
                break ;

            case 13:  // turns on the both intake motors
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, getPower(), getPower(), getDuration()), true);            
                break ;                

            case 14:  // retracts and deploys the intake arm
                // add the waits into place between deploying and retracting
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.DEPLOY), false);  
                addAction(new DelayAction(ctrl.getRobot(), 3.0)) ;
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.RETRACT), false);  
                break ;

            //
            // Numbers 20 - 29 are for the CONVEYOR
            //
            case 20:   
                // Run the intake conveyor motor
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, getPower(), 0.0, getDuration()), true) ;
                break ;

            case 21:   
                // Run the shooter conveyor motor
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, 0.0, getPower(), getDuration()), true) ;
                break ;     
                
            case 22:   
                // Run both the shooter conveyor motor and the intake conveyor motor
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, getPower(), getPower(), getDuration()), true) ;
                break ;     
                
            case 23:   
                // Run the shooter conveyor motor
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, 0.0, getPower(), getDuration()), true) ;
                break ;  

            case 24:
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, true), true);
                addAction(new DelayAction(getAutoController().getRobot(), 3.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, false), true);                
                break ;
            
            //
            // Numbers 30 - 39 are for the SHOOTER
            //
            case 30: // power to both wheel motors; position to hood motor
                addSubActionPair(shooter, new SetShooterAction(shooter, getPower(), getPower(), getPosition()), false);
                break ;

            case 31: // power to wheel motor #1
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), getDuration()), true) ;
                break ;
        
            case 32: // power to wheel motor #2
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), getDuration()), true) ;
                break ;
                
            case 33: // power to both wheel motors
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), getDuration()), true) ;
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), getDuration()), true) ;
                break ;

            case 34:
                addSubActionPair(shooter.getHoodMotor(), new MotorEncoderPowerAction(shooter.getHoodMotor(), getPower(), getDuration()), true) ;
            break ;            
                

            // case 34: // position to hood motor
            //     addSubActionPair(shooter, new SetShooterAction(shooter, 0.0, 0.0, getPosition()), false);
            //     break ;

            // //
            // // Numbers 40 - 49 are for the LIMELIGHT/TURRET/TARGETTRACKER
            // //
            // case 40: // follow the limelight!
            //     addSubActionPair(turret, new FollowTargetAction(turret, tracker), false);
            //     break ;

            case 41:
                addSubActionPair(turret, new MotorEncoderPowerAction(turret, getPower(), getDuration()), true);
                break ;

            // //
            // // Numbers 50 - 59 are for the CLIMBER
            // //
            // case 50:        
            //     if (caction_ == null) {
            //         caction_ = new ClimbAction(climber, db) ;
            //     }
            //     addSubActionPair(climber, caction_, true);      
            //     break ;

            //
            // Numbers 60 - 69 are for the GPMSubsystem
            //
            case 60:
                addSubActionPair(gpm, new GPMStartCollectAction(gpm), false) ;
                addAction(new DelayAction(ctrl.getRobot(), getDuration())) ;
                addSubActionPair(gpm, null, false);
                break ;

            // //
            // // Numbers 100+ are for the whole-robot; gpm; etc
            // //
            case 100:  
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, 1.0, 1.0, 3000), false) ;      
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), 3000), false) ;
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), 3000), true) ;
                break ;      

        }
    }
}
