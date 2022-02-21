package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMFireAction extends Action {

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

    private final String LoggerName = "fire-action" ;

    private GPMSubsystem sub_;
    private TargetTrackerSubsystem target_tracker_ ;
    private TankDriveSubsystem db_ ;
    private TurretSubsystem turret_ ;

    private ShootParams shoot_params_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;
    private boolean shoot_params_valid_ ;

    private final double db_velocity_threshold_ ;
    private final double shooter_velocity_threshold_ ;
    private final double hood_position_threshold_ ;

    private int logger_id_ ;
    
    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, 
            TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;
        turret_ = turret ;

        logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem(LoggerName) ;

        double value ;
        value = sub_.getSettingsValue("fire-action:db_vel_threshold").getDouble() ;
        db_velocity_threshold_ = value;

        value = sub_.getSettingsValue("fire-action:shooter_vel_threshold").getDouble() ;
        shooter_velocity_threshold_ = value;

        value = sub_.getSettingsValue("fire-action:hood_pos_threshold").getDouble() ;
        hood_position_threshold_ = value;

        // figure out intake and shooter doubles -> get from params
        conveyor_shoot_action_ = new ConveyorShootAction(sub_.getConveyor()) ; 
        
        // These are initial values for shooing params.  These just get the shooter spinning up
        shoot_params_ = new ShootParams(1000, 1000, 7.0) ;
        shoot_params_valid_ = false ;

        shooter_action_ = new SetShooterAction(sub_.getShooter(), shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        // set shooter to start, well, shooting...
        // This gets the shooter motors running
        sub_.getShooter().setAction(shooter_action_, true) ;

    }

    @Override
    public void run() throws Exception {
        super.run();

        boolean shooterReady, dbready ;

        if (target_tracker_.hasVisionTarget()) {
            //
            // We have a target, so compute a new set of parameters for the shooter and assign
            // to the shooter.
            //
            computeShooterParams(target_tracker_.getDistance()) ;
            shooter_action_.update(shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
            shoot_params_valid_ = true ;
        }
        else {
            //
            // We reset these here to be sure that since we lost the target, we want to force a new
            // set of shooting parametesr to be computed before we are ready to shoot.
            //
            shoot_params_valid_ = false ;
        }

        shooterReady = isShooterReady() ;
        dbready = Math.abs(db_.getVelocity()) < db_velocity_threshold_ ;

        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, logger_id_) ;
        logger.add("FireAction: adjusting shooter") ;
        logger.add("w1", shoot_params_.v1_).add("w2", shoot_params_.v2_).add("hood", shoot_params_.hood_) ;
        logger.endMessage();
        
        logger.startMessage(MessageType.Debug, logger_id_) ;
        logger.add("FireAction: ready") ;
        logger.add("shooter", shooterReady).add("turret", turret_.isReadyToFire()).add("db", dbready).add("tracker", target_tracker_.hasVisionTarget()) ;
        logger.endMessage();        
        
        // if the
        //  * shooter is up to speed
        //  * db is stopped
        //  * target tracker sees the target
        //  * turret is aimed & ready to fire
        // then, let the conveyor push cargo into the shooter
        if (shooterReady && dbready && target_tracker_.hasVisionTarget() && turret_.isReadyToFire()) 
        {
            if (sub_.getConveyor().getAction() != conveyor_shoot_action_)
                sub_.getConveyor().setAction(conveyor_shoot_action_, true) ; 
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
        if (!shoot_params_valid_)
        {
            //
            // We have nothing to compare to, so we cannot be ready
            //
            return false ;
        }

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

    public void computeShooterParams(double dist) {
    
        // TODO: compute some emperical stuff to control the shooter
        double v1 = 10.0 * dist ;
        double v2 = 10.0 * dist ;
        double hood = 1.0 * dist ;

        shoot_params_ = new ShootParams(v1, v2, hood) ;
    }
}