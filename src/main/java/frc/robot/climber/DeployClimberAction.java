package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;

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

        if (st == DeployState.Deployed) {
            deploy_position_ = sub_.getSettingsValue("deploy-action:position").getInteger() ;        
            goto_ = new MotorEncoderGotoAction(sub_.getWindmillMotor(), deploy_position_, true) ;
        }
        else {
            goto_ = new MotorEncoderGotoAction(sub_.getWindmillMotor(), 0.0, true) ;
        }
    }

    @Override
    public void start() {
        sub_.getWindmillMotor().setAction(goto_, true) ;
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