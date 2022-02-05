package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;

public class GPMFireAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;
    
    public GPMFireAction(GPMSubsystem sub) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        // figure out intake and shooter doubles -> get from params
        conveyor_shoot_action_ = new ConveyorShootAction(sub_.getConveyor(), 0.0, 0.0) ; 
        
        // figure out what w1, w2, and hood are from -> get from params file
        shooter_action_ = new SetShooterAction(sub_.getShooter(), 0.0, 0.0, 0.0) ;

    }

    @Override
    public void start() throws Exception {
        super.start();

        // set shooter to start... shooting...
        sub_.getShooter().setAction(shooter_action_) ;

    }

    @Override
    public void run() throws Exception {
        super.run();

        if (conveyor_shoot_action_.isDone()) {
            // set the shooter hood thing to "off" position
            sub_.getShooter().setAction(new SetShooterAction(sub_.getShooter(), 0.0, 0.0, 0.0)) ;
            setDone();
        }

    }

    @Override
    public void cancel() {
        super.cancel();

        conveyor_shoot_action_.cancel();
        shooter_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMFireAction";
    }

}