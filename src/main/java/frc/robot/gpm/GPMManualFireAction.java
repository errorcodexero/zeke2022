package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;

public class GPMManualFireAction extends Action {
    private GPMSubsystem sub_ ;
    private SetShooterAction shoot_action_ ;
    private ConveyorShootAction conveyor_shoot_ ;
    private double w1_ ;
    private double w2_ ;
    private double hood_ ;
    private boolean is_conveyor_on_ ;
    private double shooter_velocity_threshold_ ;
    private double hood_position_threshold_ ;

    public GPMManualFireAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        double value ;
        sub_ = sub ;

        w1_ = sub_.getSettingsValue("manual-fire-action:w1").getDouble() ;
        w2_ = sub_.getSettingsValue("manual-fire-action:w2").getDouble() ;
        hood_ = sub_.getSettingsValue("manual-fire-action:hood").getDouble() ;

        shoot_action_ = new SetShooterAction(sub_.getShooter(), w1_, w2_, hood_) ;
        conveyor_shoot_ = new ConveyorShootAction(sub_.getConveyor()) ;
        
        value = sub_.getSettingsValue("fire-action:shooter_vel_threshold").getDouble() ;
        shooter_velocity_threshold_ = value;

        value = sub_.getSettingsValue("fire-action:hood_pos_threshold").getDouble() ;
        hood_position_threshold_ = value;

        is_conveyor_on_ = false ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getShooter().setAction(shoot_action_, true) ;
        sub_.getConveyor().cancelAction();
        is_conveyor_on_ = false ;
    }

    @Override
    public void run() {

        if (is_conveyor_on_) {
            if (conveyor_shoot_.isDone()) {
                System.out.println("manual shoot done") ;
                sub_.getShooter().cancelAction();
                setDone() ;
            }
        }
        else {
            if (isShooterReady()) {
                sub_.getConveyor().setAction(conveyor_shoot_, true) ;
                is_conveyor_on_ = true ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMManualFireAction" ;
    }

    private boolean isShooterReady() {

        double w1 = sub_.getShooter().getWheelMotor1().getVelocity() ;
        double w2 = sub_.getShooter().getWheelMotor2().getVelocity() ;
        double hood = sub_.getShooter().getHoodMotor().getPosition() ;

        // find "deltas" between the actual (w whatever) and the target (shoot_params_.v whatever) 
        double dw1 = Math.abs(w1 - w1_) ;
        double dw2 = Math.abs(w2 - w2_) ;
        double dhood = Math.abs(hood - hood_) ;

        double p1 = dw1 / w1_ * 100 ;
        double p2 = dw2 / w2_ * 100 ;

        // return whether or not all the deltas are under the thresholds
        return p1 < shooter_velocity_threshold_ && p2 < shooter_velocity_threshold_ && dhood < hood_position_threshold_ ;
    }
}
