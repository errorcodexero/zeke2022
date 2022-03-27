package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ZekeIntakeOffAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private boolean intake_down_ ;

    public ZekeIntakeOffAction(ZekeIntakeSubsystem subsystem, boolean intake_down) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        intake_down_ = intake_down ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setLeftCollectorPower(0);
        subsystem_.setRightCollectorPower(0);
        if (!intake_down_)
            subsystem_.retractIntake();
        setDone();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        String str = prefix(indent) + "IntakeOffAction "  ;
        if (intake_down_)
            str += " (leave intake down)" ;

        return str ;
    }
    
}
