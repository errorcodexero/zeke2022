package frc.robot.conveyor;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;

public class ConveyorPowerAction extends Action {
    private ConveyorSubsystem subsystem_;

    private double intake_ ; 
    private double shooter_ ;
    private double duration_ ;
    private double start_ ;
    private boolean timed_ ;

    public ConveyorPowerAction(ConveyorSubsystem subsystem, double intake, double shooter)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        intake_ = intake ;
        shooter_ = shooter ;
        duration_ = 0.0 ;
        timed_ = false ;
    }

    public ConveyorPowerAction(ConveyorSubsystem subsystem, double intake, double shooter, double duration)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        intake_ = intake ;
        shooter_ = shooter ;
        duration_ = duration ;
        timed_ = true ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setMotorsPower(intake_, shooter_);

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
            subsystem_.setMotorsPower(0.0, 0.0);
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        try {
            subsystem_.setMotorsPower(0.0, 0.0);
        } catch (BadMotorRequestException | MotorRequestFailedException e) {
        }

    }

    @Override
    public String toString(int indent) {
        String str = prefix(indent) + "ConveyorPowerAction intake=" + intake_ + ", shooter=" + shooter_  ;
        if (timed_)
            str += ", duration= " + duration_ ;

        return str ;
    }
}
