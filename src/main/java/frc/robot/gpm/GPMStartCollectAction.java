package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorCollectAction;
import frc.robot.intake.ZekeIntakeOnAction;
import frc.robot.shooter.SetShooterAction;

public class GPMStartCollectAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorCollectAction conveyor_collect_action_ ;
    private ZekeIntakeOnAction intake_on_action_ ;
    private SetShooterAction shoot_action_ ;

    public GPMStartCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        conveyor_collect_action_ = new ConveyorCollectAction(sub_.getConveyor()) ;
        intake_on_action_ = new ZekeIntakeOnAction(sub_.getIntake()) ;
    }

    public GPMStartCollectAction(GPMSubsystem sub, SetShooterAction act) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        shoot_action_ = act ;
        conveyor_collect_action_ = new ConveyorCollectAction(sub_.getConveyor()) ;
        intake_on_action_ = new ZekeIntakeOnAction(sub_.getIntake()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        if (sub_ .getConveyor().isFull()) {
            setDone(); 
        }
        else {
            // collect on action -> intake
            sub_.getIntake().setAction(intake_on_action_, true) ;

            // collect on action -> conveyor
            sub_.getConveyor().setAction(conveyor_collect_action_, true) ;

            if (shoot_action_ != null)
                sub_.getShooter().setAction(shoot_action_, true) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run();

        // IF conveyor is DONE:
        if (conveyor_collect_action_.isDone()) {
            // intake -> collect off action
            intake_on_action_.cancel();
            setDone();
        }
    }

    @Override
    public void cancel() {
        super.cancel();

        // conveyor_collec_action_ => set cancel
        conveyor_collect_action_.cancel() ;

        // intake_collect_action_ => set cancel
        intake_on_action_.cancel();

        if (shoot_action_ != null)
            shoot_action_.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMCollectAction";
    }

}