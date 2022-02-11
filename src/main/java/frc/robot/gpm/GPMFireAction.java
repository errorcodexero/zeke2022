package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMFireAction extends Action {

    public static final String SubsystemName = "gpm" ;

    private GPMSubsystem sub_;
    private TargetTrackerSubsystem target_tracker_ ;
    private TankDriveSubsystem db_ ;
    private TurretSubsystem turret_ ;

    private ShootParams shoot_params_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;

    // Butch: These are just variables, nothing special in the code, lets follow our standard convention
    private final double db_velocity_threshold_ ;
    private final double shooter_velocity_threshold_ ;
    private final double hood_position_threshold_ ;
    
    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, 
            TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;

        double index ;
        index = sub_.getSettingsValue("fire-action:db_vel_threshold").getDouble() ;
        db_velocity_threshold_ = index;

        index = sub_.getSettingsValue("fire-action:shooter_vel_threshold").getDouble() ;
        shooter_velocity_threshold_ = index;

        index = sub_.getSettingsValue("fire-action:hood_pos_threshold").getDouble() ;
        hood_position_threshold_ = index;

        // figure out intake and shooter doubles -> get from params
        conveyor_shoot_action_ = new ConveyorShootAction(sub_.getConveyor(), 0.0) ; 
        
        // figure out what w1, w2, and hood default vals are from -> get from params file        
        shoot_params_ = new ShootParams(0.0, 0.0, 0.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        // set shooter to start, well, shooting...
        sub_.getShooter().setAction(shooter_action_, true) ;

    }

    @Override
    public void run() throws Exception {
        super.run();

        // prep shooters; keep it ready and up to speed
        shoot_params_ = calculate(shoot_params_, target_tracker_) ;
        sub_.getShooter().setAction(new SetShooterAction(sub_.getShooter(), 
            shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_)) ;        
        
        // if the
        //  * shooter is up to speed
        //  * db is stopped
        //  * target tracker sees the target
        //  * turret is aimed & ready to fire
        // then, let the conveyor push cargo into the shooter
        if (isShooterReady() && Math.abs(db_.getVelocity()) < db_velocity_threshold_ 
            && target_tracker_.hasVisionTarget() && turret_.isReadyToFire()) 
        {
            // TODO: once the "shooter" param is removed in the ShootAction class, get rid of the 1.0 arg passed in
            sub_.getConveyor().setAction(new ConveyorShootAction(sub_.getConveyor(), 1.0)) ; 
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

    
    boolean isShooterReady() {
        // find actual velocities/positions
        double w1 = sub_.getShooter().getWheelMotor1().getVelocity() ;
        double w2 = sub_.getShooter().getWheelMotor2().getVelocity() ;
        double hood = sub_.getShooter().getHoodMotor().getPosition() ;

        // find "deltas" between the actual (w whatever) and the target (shoot_params_.v whatever) 
        double dw1 = Math.abs(w1 - shoot_params_.v1_) ;
        double dw2 = Math.abs(w2 - shoot_params_.v2_) ;
        double dhood = Math.abs(hood - shoot_params_.hood_) ;

        // return whether or not all the deltas are under the thresholds
        boolean amIReallyReady = dw1 < shooter_velocity_threshold_ && dw2 < shooter_velocity_threshold_ && dhood < hood_position_threshold_ ;
        return  amIReallyReady ;
    }

    public ShootParams calculate(ShootParams shoot_params, TargetTrackerSubsystem target_tracker) {
        
        double dist_ = target_tracker.getDistance() ;

        // TODO: do some cool math! probably get the values/equation from testing.
        // the following "times distance" is essentially junk and just a placeholder for something more
        //      sophisticated
        shoot_params.v1_ = 1000.0 * dist_ ;
        shoot_params.v2_ = 1000.0 * dist_ ;
        shoot_params.hood_ = 10 * dist_ ;

        return shoot_params ;
    }


    // shoot params class
    // used so I can return 3 values thru one var in the "calculate" class  :)
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