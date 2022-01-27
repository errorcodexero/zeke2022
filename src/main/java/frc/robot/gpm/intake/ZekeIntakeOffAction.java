package frc.robot.gpm.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class ZekeIntakeOffAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double collector_motor_a_power_;
    private double collector_motor_b_power_;

    public ZekeIntakeOffAction(ZekeIntakeSubsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        collector_motor_a_power_ = subsystem_.getSettingsValue("hw:collector:motor-a:power").getDouble();
        collector_motor_b_power_ = subsystem_.getSettingsValue("hw:collector:motor-b:power").getDouble();

        
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
        logger.add("start") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", 0.0) ;
        logger.endMessage();

        if (!isDone())
            subsystem_.setCollectorPower(collector_motor_a_power_,collector_motor_b_power_);
        else
            subsystem_.setCollectorPower(0.0, 0.0);
            subsystem_.setSolenoidPower(false);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
        logger.add("run") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", 0.0) ;
        logger.endMessage();

        if (isDone())
            subsystem_.setCollectorPower(0.0,0.0);
            subsystem_.setSolenoidPower(false);
    }

    @Override
    public void cancel() {
        super.cancel() ;

        MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
        logger.add("cancel") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", 0.0) ;
        logger.endMessage();

        try {
            subsystem_.setCollectorPower(0.0,0.0) ;
            subsystem_.setSolenoidPower(false);
        }
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOffAction" ;
    }
    
}
