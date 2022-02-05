package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.intake.ZekeIntakeSubsystem;

public class GPMCollectAction extends Action {
    private GPMSubsystem sub_;
    private ZekeIntakeSubsystem intake_ ;
    private ConveyorSubsystem conveyor_ ;
    
    public GPMCollectAction(GPMSubsystem sub) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
        
        // // properly declare color sensor to declare this properly!
        intake_ = sub.getIntake() ;
        
        conveyor_ = sub.getConveyor();

    }

    @Override
    public void start() throws Exception {
        super.start();
        // collect on action -> intake
        intake_.setAction(new ZekeIntakeOnAction(intake_)) ;

        // ?? conveyor -> prepare to recieve

        // collect on action -> conveyor
        // what power should I set to intake and shooter here? -> get from params file!
        conveyor_.setAction(new ConveyorShootAction(conveyor_, 1.0, 0.0)) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

       // IF conveyor is DONE:
       //   intake -> collect off action
       //   ?? conveyor -> 
    }

    @Override
    public void cancel() {
        super.cancel();

      // conveyor.cancel()
      // intake -> collect off action
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMCollectAction";
    }

}