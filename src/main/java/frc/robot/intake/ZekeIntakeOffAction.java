package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class ZekeIntakeOffAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double in_speed_;
    private double out_slow_;
    private double out_fast_;
    private double stopped_;

    public ZekeIntakeOffAction(ZekeIntakeSubsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;

        in_speed_ = subsystem_.getSettingsValue("hw:collector:in_speed").getDouble();
        out_slow_ = subsystem_.getSettingsValue("hw:collector:out_slow").getDouble();
        out_fast_ = subsystem_.getSettingsValue("hw:collector:out_fast").getDouble();
        stopped_ = subsystem_.getSettingsValue("hw:collector:stopped").getDouble();
        
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
            subsystem_.setCollectorPower(in_speed_,in_speed_);
        else
            subsystem_.setCollectorPower(stopped_, stopped_);
            subsystem_.retractIntake();
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
            subsystem_.setCollectorPower(stopped_,stopped_);
            subsystem_.retractIntake();
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
            subsystem_.setCollectorPower(stopped_,stopped_) ;
            subsystem_.retractIntake();
        }
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOffAction" ;
    }
    
}
