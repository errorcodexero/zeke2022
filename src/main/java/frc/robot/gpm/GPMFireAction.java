package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMFireAction extends Action {
   
    private enum State {
        //
        // Idle, action not running yet
        IDLE,

        //
        // Waiting on all conditions to be ready.  This means we see the target with the target
        // tracker.  The drive base is stopped, the turret is aligned to the target, the shooter hood
        // is at the right angle, and the shooter wheels are runing at the right speed.
        //
        WAITING,

        //
        // We have deployed the action to the conveyor to push all balls to the shooter sheets
        //
        SHOOTING,

        //
        // The conveyor is done deliver balls, we are finishing the fire action by leaving the
        // shooter sheels running as we go
        //
        FINISHING
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

    private GPMSubsystem sub_;
    private TargetTrackerSubsystem target_tracker_ ;
    private TankDriveSubsystem db_ ;
    private TurretSubsystem turret_ ;

    private ShootParams shoot_params_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;


    private final double db_velocity_threshold_ ;
    private final double shooter_velocity_threshold_ ;
    private final double hood_position_threshold_ ;

    private int fire_action_id_ ;

    private boolean shoot_params_valid_ ;

    private double shutdown_start_time_ ;
    private double shutdown_duration_ ;

    private State state_ ;
    
    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, 
            TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;
        turret_ = turret ;
 
        shutdown_duration_ = sub_.getSettingsValue("fire-action:shutdown-delay").getDouble() ;

        fire_action_id_ = sub.getRobot().getMessageLogger().registerSubsystem("fire-action") ;

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


        shooter_action_ = new SetShooterAction(sub_.getShooter(), shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;

        state_ = State.IDLE ;

        // is_conveyor_on_ = false ;
        // is_shutdown_mode_ = false ;
        shoot_params_valid_ = false ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        //
        // We have started and are waiting to be ready to shoot
        //
        state_ = State.WAITING ;

        //
        // We have no valid shooting parameters
        //
        shoot_params_valid_ = false ;

        // shoot_params_valid_ = false ;
        // is_conveyor_on_ = false ;

        // set shooter to start, well, shooting...
        // This gets the shooter motors running
        sub_.getShooter().setAction(shooter_action_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        boolean has_target  = false;
        boolean shooter_ready = false ;
        boolean turret_ready = false ;
        boolean db_ready = false;

        switch(state_) {
            case IDLE:
                // Do nothing
                break ;

            case WAITING:  
            
                has_target = target_tracker_.hasVisionTarget() ;

                if (sub_.getConveyor().getBallCount() == 0) {
                    //
                    // This should never happen
                    //
                    shutdown_start_time_ = sub_.getRobot().getTime() ;
                    state_ = State.FINISHING ;
                }
                else if (has_target) {
                    //
                    // We have a target, 
                    //
               
                    //
                    // Compute a set of shooter parameters
                    //
                    shoot_params_valid_ = computeShooterParams(target_tracker_.getDistance()) ;

                    if (shoot_params_valid_) {
                        //
                        // Update the shooter with the current shooting parameters
                        //
                        shooter_action_.update(shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;

                        //
                        // See if the shooter is ready
                        //
                        shooter_ready = isShooterReady() ;

                        //
                        // See if the drive base is ready
                        //
                        db_ready = Math.abs(db_.getVelocity()) < db_velocity_threshold_ ;

                        //
                        // See if the turret is ready
                        //
                        turret_ready = turret_.isReadyToFire() ;

                        //
                        // check the collect button, which is a shoot button now
                        //
                        boolean shootButton = DriverStation.getStickButton(2, 3) ;
                    
                        // if the
                        //  * shooter is up to speed
                        //  * db is stopped
                        //  * target tracker sees the target
                        //  * turret is aimed & ready to fire
                        // then, let the conveyor push cargo into the shooter
                        if ((shooter_ready && db_ready && target_tracker_.hasVisionTarget() && turret_ready) || shootButton)
                        {
                            shooter_action_.startPlot();
                            sub_.getConveyor().setAction(conveyor_shoot_action_, true) ;
                            state_ = State.SHOOTING ;
                        }
                    }

                }
                else {
                    //
                    // We have no vision target, therefore our shooting parameters are invalid
                    //
                    shoot_params_valid_ = false ;
                }            
                break ;

            case SHOOTING:
                //
                // We are shooting, shoot til all balls done
                //

                if (sub_.getConveyor() == null || sub_.getConveyor().getAction().isDone()) {
                    //
                    // The conveyor has delivered all balls, go to finishing state which
                    // keeps the wheels running while the last ball leaves the robot
                    //
                    shutdown_start_time_ = sub_.getRobot().getTime() ;
                    state_ = State.FINISHING ;
                }
                break ;            

            case FINISHING:
                if (sub_.getRobot().getTime() - shutdown_start_time_ > shutdown_duration_) {
                    shooter_action_.stopPlot();
                    sub_.getShooter().setAction(null, true) ;
                    state_ = State.IDLE ;                    setDone() ;

                }
                break ;
        }

        //
        // Log the current fire action state
        //

        logger.startMessage(MessageType.Debug, fire_action_id_) ;
        logger.add("state", state_.toString()) ;

        if (state_ == State.WAITING) {
            if (has_target) {
                logger.add("spvalid", shoot_params_valid_) ;
                if (shoot_params_valid_) {
                    logger.add("thood", shoot_params_.hood_) ;
                    logger.add("tvelocity", shoot_params_.v1_) ;
                    logger.add("ahood", sub_.getShooter().getHoodMotor().getPosition()) ;
                    logger.add("am1", sub_.getShooter().getWheelMotor1().getVelocity()) ;
                    logger.add("am2", sub_.getShooter().getWheelMotor2().getVelocity()) ;
                }
                logger.add("shooterready", shooter_ready) ;
            }
            else {
                logger.add(", no target") ;
            }

            logger.add("dbready", db_ready) ;
            logger.add("turretready", turret_ready) ;
        }
        logger.endMessage();
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
        boolean ret = false ;

        if (shoot_params_valid_ && shoot_params_ != null) {

            // find actual velocities/positions
            double w1 = sub_.getShooter().getWheelMotor1().getVelocity() ;
            double w2 = sub_.getShooter().getWheelMotor2().getVelocity() ;
            double hood = sub_.getShooter().getHoodMotor().getPosition() ;

            // find "deltas" between the actual (w whatever) and the target (shoot_params_.v whatever) 
            double dw1 = Math.abs(w1 - shoot_params_.v1_) ;
            double dw2 = Math.abs(w2 - shoot_params_.v2_) ;
            double dhood = Math.abs(hood - shoot_params_.hood_) ;

            // return whether or not all the deltas are under the thresholds
            ret = dw1 < shooter_velocity_threshold_ && dw2 < shooter_velocity_threshold_ && dhood < hood_position_threshold_ ;
        }
        return  ret ;
    }

    public boolean computeShooterParams(double dist) {
        boolean ret = true ;

        double vel = 0.4992 * dist * dist - 38.828 * dist + 3500.5 ;        
        double hood = 0.156 * dist ;

        if (hood < 1.0 || hood > 22.0) {
            ret = false ;
        }
        else {
            shoot_params_ = new ShootParams(vel, vel, hood) ;
        }

        return ret ;
    }
}
