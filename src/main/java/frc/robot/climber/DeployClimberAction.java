package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;

public class DeployClimberAction extends Action {
    private ClimberSubsystem sub_ ;
    private int deploy_position_ ;
    private MotorEncoderGotoAction goto_ ;
    private DeployState state_ ;

    public enum DeployState {
        Deployed,
        Stowed
    } ;
    
    public DeployClimberAction(ClimberSubsystem sub, DeployState st) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        
        sub_ = sub ;
        state_ = st ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        
        if (state_ == DeployState.Deployed) {
            deploy_position_ = sub_.getSettingsValue("deploy-action:deployed").getInteger() ;        
        }
        else {
            deploy_position_ = sub_.getSettingsValue("deploy-action:stowed").getInteger() ;    
        }

        goto_ = new MotorEncoderGotoAction(sub_.getWindmillMotor(), deploy_position_, true) ;

        sub_.getWindmillMotor().setAction(goto_, true) ;
        if (state_ == DeployState.Stowed) {
            sub_.changeClamp(WhichClamp.CLAMP_A, GrabberState.CLOSED);
            sub_.changeClamp(WhichClamp.CLAMP_B, GrabberState.CLOSED);
        }
    }

    @Override 
    public void run() {
        if (goto_.isDone()) {
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        goto_.cancel();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "DeployClimberAction" ;
    }
}