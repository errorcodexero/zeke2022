package frc.robot.gpm.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.gpm.intake.ZekeIntakeSubsystem.BallColors;

public class ZekeIntakeOnAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double collector_motor_a_power_;
    private double collector_motor_b_power_;
    private int blocked_count;
    private States intake_state;

    private enum States {
        BLOCKED,
        COLLECT,
        EMPTY,
    }

    public ZekeIntakeOnAction(ZekeIntakeSubsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        collector_motor_a_power_ = subsystem_.getSettingsValue("hw:collector:motor-a:power").getDouble();
        collector_motor_b_power_ = subsystem_.getSettingsValue("hw:collector:motor-b:power").getDouble();
        
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setSolenoidPower(true);
        subsystem_.setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
        blocked_count = 0;
        intake_state = States.EMPTY;
    }
    
    @Override
    public void run() throws Exception {
        super.run() ;

        if (subsystem_.isIntakeBlocked()) {
            if (blocked_count >= 10) {
                if (intake_state == States.BLOCKED) {
                    subsystem_.clearIntake();
                    intake_state = States.EMPTY;
                    blocked_count = 0;
                }
            } else {
                blocked_count++;
            }
        } else {
            if (subsystem_.getLeftBallColor() == BallColors.NOTHING && subsystem_.getRightBallColor() == BallColors.NOTHING) {
                blocked_count = 0;
                intake_state = States.EMPTY;
            } else {
                if (subsystem_.getLeftBallColor() == BallColors.SAME || subsystem_.getRightBallColor() == BallColors.SAME ) {
                    intake_state = States.COLLECT;
                }
            }
        }
        

    }

    @Override
    public void cancel() {
        super.cancel();

        try {
            subsystem_.setSolenoidPower(false);
            subsystem_.setCollectorPower(0.0,0.0);
        } 
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOnAction" ;
    }

}