package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorOnAction;
import frc.robot.conveyor.ConveyorPrepareToReceiveAction;
import frc.robot.conveyor.ConveyorStopAction;
import frc.robot.intake.ZekeIntakeOffAction;
import frc.robot.intake.ZekeIntakeOnAction;

public class GPMCollectAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorOnAction conveyor_on_action_ ;
    private ZekeIntakeOnAction intake_on_action_ ;

    public GPMCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        // TODO: find proper intake and shooter double values from params
        conveyor_on_action_ = new ConveyorOnAction(sub_.getConveyor(), 0.0, 0.0) ; 

        intake_on_action_ = new ZekeIntakeOnAction(sub_.getIntake()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        // collect on action -> intake
        sub_.getIntake().setAction(new ZekeIntakeOnAction(sub_.getIntake())) ;

        // ?? prepare to recieve -> conveyor
        sub_.getConveyor().setAction(new ConveyorPrepareToReceiveAction(sub_.getConveyor())) ;

        // collect on action -> conveyor
        sub_.getConveyor().setAction(conveyor_on_action_) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        // IF conveyor is DONE:
        if (conveyor_on_action_.isDone()) {
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
        conveyor_on_action_.cancel() ;

        // intake_collect_action_ => set cancel
        intake_on_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMCollectAction";
    }

}