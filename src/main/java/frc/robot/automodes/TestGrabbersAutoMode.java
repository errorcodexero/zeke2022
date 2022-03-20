package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

import frc.robot.climber.ChangeClampAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class TestGrabbersAutoMode extends ZekeAutoMode {
    public TestGrabbersAutoMode(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name) ;

        ZekeSubsystem robot = (ZekeSubsystem) ctrl.getRobot().getRobotSubsystem() ;
        ClimberSubsystem climber = robot.getClimber() ;
        MotorEncoderSubsystem windmill = climber.getWindmillMotor() ;
        MotorEncoderGotoAction gplus45 = new MotorEncoderGotoAction(windmill, 45.0, true) ;
        MotorEncoderGotoAction gminus45 = new MotorEncoderGotoAction(windmill, -45.0, true) ;
        MotorEncoderGotoAction gzero = new MotorEncoderGotoAction(windmill, 0.0, true) ;
        
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
    }    
}
