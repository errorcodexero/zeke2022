package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveScrubCharAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.Zeke2022;
import frc.robot.climber.ChangeClampAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.ChangeClampTo;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.conveyor.ConveyorBypassAction;
import frc.robot.conveyor.ConveyorExitAction;
import frc.robot.conveyor.ConveyorPowerAction;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.gpm.GPMShooterTestAction;
import frc.robot.intake.ZekeIntakeArmAction;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.intake.ZekeIntakePowerAction;
import frc.robot.intake.ZekeIntakeSubsystem;
import frc.robot.shooter.SetShooterAction;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeTestModeAuto extends TestAutoMode {
    private boolean has_climber_ ;

    public ZekeTestModeAuto(ZekeAutoController ctrl) throws Exception {
        super(ctrl, "Zeke2022-Test-Mode");

        Zeke2022 robot = (Zeke2022)ctrl.getRobot() ;
        has_climber_ = robot.hasClimber() ;

        ZekeSubsystem zeke = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        ZekeIntakeSubsystem intake = zeke.getGPMSubsystem().getIntake() ;   
        ConveyorSubsystem conveyor = zeke.getGPMSubsystem().getConveyor() ;
        ShooterSubsystem shooter = zeke.getGPMSubsystem().getShooter() ;
        TurretSubsystem turret = zeke.getTurret() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        ClimberSubsystem climber = zeke.getClimber() ;

        switch (getTestNumber()) {
            //
            //////////////////////////////////////////////////////////////////////////////////
            // DRIVEBASE                                                                    //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 0: // Drive straight, used to test and get Kv number
                addSubActionPair(db, new TankDrivePowerAction(db, getPower(), getPower(), getDuration()), true);
                break;

            case 1:
                addSubActionPair(db, new TankDriveScrubCharAction(db, 0.5, 3600), true);
                break ;

            case 2:
                addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_4_p1", false), true) ;
                addAction(new DelayAction(ctrl.getRobot(), 2.0));   
                addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_4_p2", true), true) ;
                addAction(new DelayAction(ctrl.getRobot(), 2.0));  
                addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_4_p3", false), true) ;
                addAction(new DelayAction(ctrl.getRobot(), 2.0));  
                addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_4_p4", true), true) ;
                addAction(new DelayAction(ctrl.getRobot(), 2.0));                                                               
                break ;  
            case 3:
                addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_2_ball_1", false), true) ;
                break ;
                        
            //
            //////////////////////////////////////////////////////////////////////////////////
            // INTAKE                                                                       //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 10:  
                // turns on the left intake motor
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, getPower(), 0.0, getDuration()), true);            
                break ;
            
            case 11:  
                // turns on the right intake motor
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, 0.0, getPower(), getDuration()), true);            
                break ;

            case 12:  
                // turns on the both intake motors
                addSubActionPair(intake, new ZekeIntakePowerAction(intake, getPower(), getPower(), getDuration()), true);            
                break ;                

            case 13:  
                // deploys and then retracts the intake
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.DEPLOY), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.RETRACT), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;      
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.DEPLOY), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.RETRACT), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;  
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.DEPLOY), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;
                addSubActionPair(intake, new ZekeIntakeArmAction(intake, ZekeIntakeArmAction.ArmPos.RETRACT), false);  
                addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;                                            
                break ;

            case 14:
                addSubActionPair(intake, new ZekeIntakeOnAction(intake), true) ;
                break ;

            //
            //////////////////////////////////////////////////////////////////////////////////
            // CONVEYOR                                                                     //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 20:   
                // Run the intake conveyor motor
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, true), true);
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, getPower(), 0.0, getDuration()), true) ;
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, false), true);
                break ;

            case 21:   
                // Run the shooter conveyor motor
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, true), true);
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, 0.0, getPower(), getDuration()), true) ;
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, false), true);                
                break ;     
                
            case 22:   
                // Run both the shooter conveyor motor and the intake conveyor motor
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, true), true);
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, getPower(), getPower(), getDuration()), true) ;
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, false), true);                
                break ;     
                
            case 24:
                // Open and then close the exit door (3 complete cycles)
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, true), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, true), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, true), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                addSubActionPair(conveyor, new ConveyorExitAction(conveyor, false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 1.0)) ;
                break ;
            
            //
            //////////////////////////////////////////////////////////////////////////////////
            // SHOOTER                                                                      //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 30:
                // Set power the the first wheel motor
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), getDuration()), true) ;
                break ;

            case 31:
                // Set the power to the second wheel motor
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), getDuration()), true) ;
                break ;
        
            case 32: 
                // Set the power to both wheel motors
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), getDuration()), true) ;
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), getDuration()), true) ;
                break ;

            case 33:
                // Set the power to the second hood motor (use with care)
                addSubActionPair(shooter.getHoodMotor(), new MotorEncoderPowerAction(shooter.getHoodMotor(), getPower(), getDuration()), true) ;              
                break ;

            case 34:
                // Set the shooter wheels to a specific velocity
                addSubActionPair(shooter, new SetShooterAction(shooter, getPower(), getPower(), shooter.getHoodMotor().getPosition()), false);
                break ;   
                
            case 35:
                // Set the hood to a specific position
                addSubActionPair(shooter, new SetShooterAction(shooter, 0.0, 0.0, getPosition()), false);
                break ;                                 

            //
            //////////////////////////////////////////////////////////////////////////////////
            // TURRET/TARGET_TRACKER                                                        //
            //////////////////////////////////////////////////////////////////////////////////
            //

            case 40:
                // Move the turret motor
                addSubActionPair(turret, new MotorEncoderPowerAction(turret, getPower(), getDuration()), true);
                break; 

            case 41:
                addSubActionPair(turret, new MotorEncoderGotoAction(turret, 45.0, true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 3.0)) ;
                addSubActionPair(turret, new MotorEncoderGotoAction(turret, -45.0, true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 3.0)) ;
                addSubActionPair(turret, new MotorEncoderGotoAction(turret, 0.0, true), true) ;                
                break ;

            case 42:
                // Have the turret follow the target
                addSubActionPair(turret, new FollowTargetAction(turret, tracker), false);
                break ;

            //
            //////////////////////////////////////////////////////////////////////////////////
            // CLIMBER                                                                      //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 50:
                // Exercise the clamps on the A end of the climber windmill
                if (has_climber_) {
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;   
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;   
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;     
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;   
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ;  

                }
                break ;

            case 51:
                // Exercise the clamps on the B end of the climber windmill
                if (has_climber_) {
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.OPEN), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                    addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, ChangeClampTo.CLOSED), false);
                    addAction(new DelayAction(ctrl.getRobot(), 2.0)) ; 
                }
                break ;

            case 52:
                // Exercise the windmill motors
                if (has_climber_) {
                    addSubActionPair(climber.getWindmillMotor(), new MotorEncoderPowerAction(climber.getWindmillMotor(), getPower(), getDuration()), true);
                }
                break ;

            //
            //////////////////////////////////////////////////////////////////////////////////
            // WHOLE ROBOT                                                                  //
            //////////////////////////////////////////////////////////////////////////////////
            //
            case 100:  
                addSubActionPair(conveyor, new ConveyorBypassAction(conveyor, true), true);
                addSubActionPair(shooter.getWheelMotor1(), new MotorEncoderPowerAction(shooter.getWheelMotor1(), getPower(), getDuration()), false) ;
                addSubActionPair(shooter.getWheelMotor2(), new MotorEncoderPowerAction(shooter.getWheelMotor2(), getPower(), getDuration()), false) ;
                addSubActionPair(conveyor, new ConveyorPowerAction(conveyor, 1.0, 1.0, getDuration()), true) ;
                break ;

            case 101:
                addSubActionPair(zeke.getTurret(), new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker()), false);
                addSubActionPair(zeke.getGPMSubsystem(), new GPMShooterTestAction(zeke.getGPMSubsystem()), true);
                break ;
        }
    }
}
