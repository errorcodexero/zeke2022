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

    private int ready_logger_id_ ;
    private int shoot_params_logger_id_ ;
    private int shoot_perf_logger_id_ ;

    private boolean is_conveyor_on_ ;
    private boolean in_shutdown_mode_ ;
    private double shutdown_start_time_ ;
    private double shutdown_duration_ ;
    
    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, 
            TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;
        turret_ = turret ;
        is_conveyor_on_ = false ;
        shutdown_duration_ = sub_.getSettingsValue("fire-action:shutdown-delay").getDouble() ;

        ready_logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("fire-action-ready") ;
        shoot_params_logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("fire-action-shoot-params") ;
        shoot_perf_logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("fire-action-shoot-performance") ;

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
        shoot_params_ = new ShootParams(5000, 5000, 7.0) ;
        shoot_params_valid_ = false ;

        shooter_action_ = new SetShooterAction(sub_.getShooter(), shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        shoot_params_valid_ = false ;
        is_conveyor_on_ = false ;

        // set shooter to start, well, shooting...
        // This gets the shooter motors running
        sub_.getShooter().setAction(shooter_action_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        boolean shooterReady, dbready ;




        if (in_shutdown_mode_) {
            if (sub_.getRobot().getTime() - shutdown_start_time_ > shutdown_duration_) {
                shooter_action_.stopPlot();
                sub_.getShooter().setAction(null, true) ;
                setDone() ;
                in_shutdown_mode_ = false ;
            }
        }

        if (is_conveyor_on_) { 
            //
            // We are shooting, shoot til all balls done
            //
            if (sub_.getConveyor() == null || sub_.getConveyor().getAction().isDone()) {
                shutdown_start_time_ = sub_.getRobot().getTime() ;
                in_shutdown_mode_ = true ;
                is_conveyor_on_ = false ;
            }
        }
        else {
            //
            // We are waiting to be ready to shoot
            //           
            if (target_tracker_.hasVisionTarget() && sub_.getConveyor().getBallCount() > 0) {
                MessageLogger logger = sub_.getRobot().getMessageLogger() ;
                
                //
                // We have a target, so compute a new set of parameters for the shooter and assign
                // to the shooter.
                //
                computeShooterParams(target_tracker_.getDistance()) ;
                shooter_action_.update(shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
                shoot_params_valid_ = true ;

                shooterReady = isShooterReady() ;
                dbready = Math.abs(db_.getVelocity()) < db_velocity_threshold_ ;

                logger.startMessage(MessageType.Debug, shoot_params_logger_id_) ;
                logger.add("Shooter Params Targets:") ;
                logger.add("dist", target_tracker_.getDistance()) ;
                logger.add("w1", shoot_params_.v1_).add("w2", shoot_params_.v2_).add("hood", shoot_params_.hood_) ;
                logger.endMessage();
                
                logger.startMessage(MessageType.Debug, ready_logger_id_) ;
                logger.add("FireAction Ready:") ;
                logger.add("shooter", shooterReady).add("turret", turret_.isReadyToFire()).add("db", dbready).add("tracker", target_tracker_.hasVisionTarget()) ;
                logger.endMessage();        

                // sub_.putDashboard("shootrdy", DisplayType.Always, shooterReady);
                // sub_.putDashboard("dbready", DisplayType.Always, dbready);
                // sub_.putDashboard("trtready", DisplayType.Always, turret_.isReadyToFire());
                // sub_.putDashboard("llready", DisplayType.Always, target_tracker_.hasVisionTarget());
                
                // if the
                //  * shooter is up to speed
                //  * db is stopped
                //  * target tracker sees the target
                //  * turret is aimed & ready to fire
                // then, let the conveyor push cargo into the shooter
                if (shooterReady && dbready && target_tracker_.hasVisionTarget() && turret_.isReadyToFire()) 
                {
                    shooter_action_.startPlot();
                    sub_.getConveyor().setAction(conveyor_shoot_action_, true) ;
                    is_conveyor_on_ = true ;
                }
            }
            else {
                //
                // We have no vision target, therefore our shooting parameters are invalid
                //
                shoot_params_valid_ = false ;
            }
        }
    }

    @Override
    public void cancel() {
        super.cancel();

        shooter_action_.stopPlot();

        conveyor_shoot_action_.cancel();
        shooter_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMFireAction";
    }

    boolean isShooterReady() {
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;

        if (!shoot_params_valid_)
        {
            logger.startMessage(MessageType.Debug, shoot_perf_logger_id_).add("shooterready: no valid params").endMessage();
            //
            // We have nothing to compare to, so we cannot be ready
            //
            return false ;
        }

        if (shoot_params_ == null) {
            logger.startMessage(MessageType.Debug, shoot_perf_logger_id_).add("shooterready: shoot params null").endMessage();            
            return  false ;
        }

        // find actual velocities/positions
        double w1 = sub_.getShooter().getWheelMotor1().getVelocity() ;
        double w2 = sub_.getShooter().getWheelMotor2().getVelocity() ;
        double hood = sub_.getShooter().getHoodMotor().getPosition() ;

        // find "deltas" between the actual (w whatever) and the target (shoot_params_.v whatever) 
        double dw1 = Math.abs(w1 - shoot_params_.v1_) ;
        double dw2 = Math.abs(w2 - shoot_params_.v2_) ;
        double dhood = Math.abs(hood - shoot_params_.hood_) ;

        logger.startMessage(MessageType.Debug, shoot_perf_logger_id_) ;
        logger.add("Shooter Performance:") ;
        logger.add("w1act", w1) ;
        logger.add("w2act", w2) ;
        logger.add("hoodact", hood) ;
        logger.add("w1delta", dw1) ;
        logger.add("w2delta", dw2) ;
        logger.add("hooddelta", dhood) ;
        logger.endMessage();

        // return whether or not all the deltas are under the thresholds
        boolean amIReallyReady = dw1 < shooter_velocity_threshold_ && dw2 < shooter_velocity_threshold_ && dhood < hood_position_threshold_ ;
        return  amIReallyReady ;
    }

    public void computeShooterParams(double dist) {
        double vel = 0.4992 * dist * dist - 38.828 * dist + 4060.5 ;
        double hood = 0.156 * dist + 0.5801 ;

        if (hood < 1.0 || hood > 22.0) {
            shoot_params_valid_ = false ;
        }

        shoot_params_ = new ShootParams(vel, vel, hood) ;
    }
}
