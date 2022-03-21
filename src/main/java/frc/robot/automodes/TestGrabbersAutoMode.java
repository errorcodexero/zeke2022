package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.motorsubsystem.MotorEncoderVelocityAction;

import frc.robot.climber.ChangeClampAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class TestGrabbersAutoMode extends ZekeAutoMode {
    public TestGrabbersAutoMode(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name) ;

        ZekeSubsystem robot = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        ClimberSubsystem climber = robot.getClimber() ;
        MotorEncoderSubsystem windmill = climber.getWindmillMotor() ;
        ShooterSubsystem shooter = robot.getGPMSubsystem().getShooter() ;

        MotorEncoderGotoAction gplus45 = new MotorEncoderGotoAction(windmill, 45.0, true) ;
        MotorEncoderGotoAction gminus45 = new MotorEncoderGotoAction(windmill, -45.0, true) ;
        MotorEncoderGotoAction gzero = new MotorEncoderGotoAction(windmill, 0.0, true) ;

        MotorEncoderTrackPositionAction ghood3 = new MotorEncoderTrackPositionAction(shooter.getHoodMotor(), "hood", 3.0) ;
        MotorEncoderTrackPositionAction ghood18 = new MotorEncoderTrackPositionAction(shooter.getHoodMotor(), "hood", 18.0) ;

        MotorEncoderVelocityAction v1 = new MotorEncoderVelocityAction(shooter.getWheelMotor1(), "w1", 2000) ;
        MotorEncoderVelocityAction v2 = new MotorEncoderVelocityAction(shooter.getWheelMotor2(), "w2", 2000) ;
        
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.OPEN), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.CLOSED), false);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.CLOSED), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.OPEN), false);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ;     
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.OPEN), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.CLOSED), false);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.CLOSED), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.OPEN), false);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ;   
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_B, GrabberState.OPEN), false);
        addSubActionPair(climber, new ChangeClampAction(climber, WhichClamp.CLAMP_A, GrabberState.CLOSED), false);

        addSubActionPair(windmill, gplus45, true);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ;   
        addSubActionPair(windmill, gminus45, true);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ;  
        addSubActionPair(windmill, gzero, true);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ;
        
        climber.cancelAction();

        addSubActionPair(shooter.getHoodMotor(), ghood3, true) ;
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 
        
        addSubActionPair(shooter.getHoodMotor(), ghood18, true) ;
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 

        addSubActionPair(shooter.getWheelMotor1(), v1, true);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 
        shooter.getWheelMotor1().cancelAction() ;

        addSubActionPair(shooter.getWheelMotor2(), v2, true);
        addAction(new DelayAction(ctrl.getRobot(),2.0)) ; 

    }    
}
