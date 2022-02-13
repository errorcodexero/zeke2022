package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;

public class ZekeIntakePowerAction extends Action {
    private ZekeIntakeSubsystem subsystem_;

    private double left_ ; 
    private double right_ ;
    private double duration_ ;
    private double start_ ;
    private boolean timed_ ;

    public ZekeIntakePowerAction(ZekeIntakeSubsystem subsystem, double left, double right)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        left_ = left ;
        right_ = right ;
        duration_ = 0.0 ;
        timed_ = false ;
    }

    public ZekeIntakePowerAction(ZekeIntakeSubsystem subsystem, double left, double right, double duration)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        left_ = left ;
        right_ = right ;
        duration_ = duration ;
        timed_ = true ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setLeftCollectorPower(left_);
        subsystem_.setRightCollectorPower(right_);

        if (timed_) {
            start_ = subsystem_.getRobot().getTime() ;
        }
        else {
            setDone() ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (subsystem_.getRobot().getTime() - start_ > duration_) {
            subsystem_.setLeftCollectorPower(0.0);
            subsystem_.setRightCollectorPower(0.0);
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        try {
            subsystem_.setLeftCollectorPower(0.0);
            subsystem_.setRightCollectorPower(0.0);
        } catch (BadMotorRequestException | MotorRequestFailedException e) {
        }

    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOffAction left=" + left_ + ", right=" + right_  + ", timed=" + timed_ ;
    }
    
}
