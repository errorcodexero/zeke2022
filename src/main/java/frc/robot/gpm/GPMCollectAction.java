package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorCollectAction;
import frc.robot.conveyor.ConveyorStopAction;
import frc.robot.intake.ZekeIntakeOffAction;
import frc.robot.intake.ZekeIntakeOnAction;

public class GPMCollectAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorCollectAction conveyor_collect_action_ ;
    private ZekeIntakeOnAction intake_on_action_ ;

    public GPMCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        // TODO: find proper intake and shooter double values from params
        conveyor_collect_action_ = new ConveyorCollectAction(sub_.getConveyor(), 0.0, 0.0) ; 

        intake_on_action_ = new ZekeIntakeOnAction(sub_.getIntake()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        // collect on action -> intake
        sub_.getIntake().setAction(new ZekeIntakeOnAction(sub_.getIntake())) ;

        // ?? prepare to recieve -> conveyor
        sub_.getConveyor().setAction(new ConveyorCollectAction(sub_.getConveyor(),1.0,0.0)) ;

        // collect on action -> conveyor
        sub_.getConveyor().setAction(conveyor_collect_action_) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        // IF conveyor is DONE:
        if (conveyor_collect_action_.isDone()) {
            // intake -> collect off action
            sub_.getIntake().setAction(new ZekeIntakeOffAction(sub_.getIntake())) ;
            
            // set the intake on action => cancel
            intake_on_action_.cancel();

            // conveyor -> stop conveyor?
            sub_.getConveyor().setAction(new ConveyorStopAction(sub_.getConveyor())) ;
        }

    }

    @Override
    public void cancel() {
        super.cancel();

        // conveyor_collec_action_ => set cancel
        conveyor_collect_action_.cancel() ;

        // intake_collect_action_ => set cancel
        intake_on_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMCollectAction";
    }

}