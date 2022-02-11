package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMFireAction extends Action {
    private GPMSubsystem sub_;
    private TargetTrackerSubsystem target_tracker_ ;
    private TankDriveSubsystem db_ ;

    private ShootParams shoot_params_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;
    
    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, 
            TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;

        // figure out intake and shooter doubles -> get from params
        conveyor_shoot_action_ = new ConveyorShootAction(sub_.getConveyor(), 0.0) ; 
        
        // figure out what w1, w2, and hood are from -> get from params file
        shooter_action_ = new SetShooterAction(sub_.getShooter(), 0.0, 0.0, 0.0) ;

    }

    @Override
    public void start() throws Exception {
        super.start();

        // set shooter to start... shooting...
        sub_.getShooter().setAction(shooter_action_, true) ;

    }

    @Override
    public void run() throws Exception {
        super.run();
        
        // TODO: get tiny numbers from the params file
        if (conveyor_shoot_action_.isDone() && Math.abs(db_.getVelocity()) < 0.001 
                && target_tracker_.hasVisionTarget() && sub_.getTurret().isReadyToFire()) {
            
            shoot_params_ = calculate(shoot_params_, target_tracker_) ;

            sub_.getShooter().setAction(new SetShooterAction(sub_.getShooter(), 
                shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_)) ;
            
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

    public ShootParams calculate(ShootParams shoot_params, TargetTrackerSubsystem target_tracker) {
        
        double dist_ = target_tracker.getDistance() ;

        // do some cool math!
        //vf^2 - vi^2 = 2ad   ,  where a is g = gravity = 9.81(m/(s*s))
        // (vi*cos(theta))^2 = 2gh   ,  where h is the height of the hub (104 inches), 
        //           minus the robot's shooter height, plus a bit so the arc's neat going in
        // vi = (2gh)^(1/2) / (cos(theta))
        // theta =
        // EDIT: I could either work out all the theoretical math or we can get out the robot and test it ...
        shoot_params.v1_ = 0.0 ;
        shoot_params.v2_ = 0.0 ;
        shoot_params.hood_ = 0.0 ;

        return shoot_params ;
    }

    // shoot params class
    // used so I can return 3 values thru one var
    private class ShootParams {
        public double v1_ ;
        public double v2_ ;
        public double hood_ ;

        public ShootParams(double v1, double v2, double hood) {
            v1_ = v1 ;
            v2_ = v2 ;
            hood_ = hood ;
        }
    }

}