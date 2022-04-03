package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMFireAction extends Action {
   
    private enum State {
        //
        // Idle, action not running yet
        IDLE,

        DELAY,

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

    private XeroTimer delay_timer_ ;

    private ShootParams shoot_params_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;

    private final double db_velocity_threshold_ ;
    private final double shooter_velocity_threshold_ ;
    private final double hood_position_threshold_ ;
    private final double gyro_threshold_ ;
    private final double accel_threshold_ ;

    private int fire_action_id_ ;

    private boolean shoot_params_valid_ ;

    private XeroTimer shutdown_start_timer_ ;

    private State state_ ;

    private boolean shooter_ready_ ;
    private boolean turret_ready_ ;
    private boolean has_target_ ;
    private boolean db_ready_ ;

    private PieceWiseLinear pwl_hood_ ;
    private PieceWiseLinear pwl_velocity_ ;

    private int go_ ;

    private int plot_id_ ;
    private double plot_start_ ;
    private Double [] plot_data_ ;
    private static String [] columns_ = { "time", "dist", "go", "target(rpm)", "l-actual(rpm)", "r-actual(rpm)" } ;

    public GPMFireAction(GPMSubsystem sub, TargetTrackerSubsystem target_tracker, TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());

        double value ;        

        delay_timer_ = new XeroTimer(sub.getRobot(), "DelayTimer", 0.01) ;

        sub_ = sub ;
        target_tracker_ = target_tracker ;
        db_ = db ;
        turret_ = turret ;
        go_ = 0 ;
 
        shutdown_start_timer_ = new XeroTimer(sub.getRobot(), "fire-action-shutdown-timer", sub_.getSettingsValue("fire-action:shutdown-delay").getDouble()) ;

        fire_action_id_ = sub.getRobot().getMessageLogger().registerSubsystem("fire-action") ;

        value = sub_.getSettingsValue("fire-action:db_vel_threshold").getDouble() ;
        db_velocity_threshold_ = value;

        value = sub_.getSettingsValue("fire-action:navx_gyro_threshold").getDouble() ;
        gyro_threshold_ = value;

        value = sub_.getSettingsValue("fire-action:navx_accel_threshold").getDouble() ;
        accel_threshold_ = value;

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

        shoot_params_valid_ = false ;

        pwl_hood_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:hood-pwl") ;
        pwl_velocity_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:gpm:fire-action:velocity-pwl") ;

        plot_id_ = sub_.initPlot("Fire") ;
    }

    public boolean distanceOk() {
        return shoot_params_valid_ ;
    }

    public boolean turretReady() {
        return turret_ready_ ;
    }

    public boolean shooterReady() {
        return shooter_ready_ ;
    }

    public boolean hasTarget() {
        return has_target_ ; 
    }

    public boolean dbReady() {
        return db_ready_ ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        go_ = 0 ;
        delay_timer_.start() ;

        //
        // We have started and are waiting to be ready to shoot
        //
        state_ = State.DELAY ;

        //
        // We have no valid shooting parameters
        //
        shoot_params_valid_ = false ;

        // shoot_params_valid_ = false ;
        // is_conveyor_on_ = false ;

        // set shooter to start, well, shooting...
        // This gets the shooter motors running
        sub_.getShooter().setAction(shooter_action_, true) ;

        sub_.startPlot(plot_id_, columns_);
        plot_start_ = sub_.getRobot().getTime() ;
        plot_data_ = new Double[columns_.length] ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        has_target_  = false;
        shooter_ready_ = false ;
        turret_ready_ = false ;
        db_ready_ = false;
        shoot_params_valid_ = false ;

        switch(state_) {
            case IDLE:
                // Do nothing
                break ;

            case DELAY:
            case WAITING:  
                has_target_ = target_tracker_.hasTarget() ;

                if (sub_.getConveyor().getBallCount() == 0) {
                    //
                    // This should never happen
                    //
                    shutdown_start_timer_.start();
                    state_ = State.FINISHING ;
                }
                else if (has_target_) {
                    //
                    // We have a target, 
                    //
               
                    //
                    // Compute a set of shooter parameters
                    //
                    shoot_params_valid_ = computeShooterParams(target_tracker_.getDistance()) ;

                    //
                    // Update the shooter with the current shooting parameters.  We do this whether or not the
                    // shooting parameters are valid because if they are not valid, we set the parameter values to
                    // be at the edge of the range.  This way, we keep the shooter wheels spun up to the best speed
                    // for when the distance comes in range.
                    //
                    shooter_action_.update(shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;

                    if (shoot_params_valid_) {
                        //
                        // See if the shooter is ready
                        //
                        shooter_ready_ = isShooterReady() ;

                        //
                        // See if the drive base is ready
                        //
                        db_ready_ = isDBReady() ;

                        //
                        // See if the turret is ready
                        //
                        turret_ready_ = turret_.isReadyToFire() ;

                        //
                        // If we are here, we have a target and the shooter params are good
                        //
                        if (shooter_ready_ && db_ready_ && turret_ready_ && state_ == State.WAITING)
                        {
                            logger.startMessage(MessageType.Info) ;
                            logger.add("shooting: ") ;
                            logger.add("distance", target_tracker_.getDistance()) ;
                            logger.add("hood", shoot_params_.hood_) ;
                            logger.add("velocity", shoot_params_.v1_) ;
                            logger.add("balls", sub_.getConveyor().getBallCount()) ;
                            logger.endMessage();

                            shooter_action_.startPlot();
                            sub_.getConveyor().setAction(conveyor_shoot_action_, true) ;
                            state_ = State.SHOOTING ;
                            go_ = 1 ;
                        }
                    }
                }
                break ;

            case SHOOTING:
                //
                // We are shooting, shoot til all balls done
                //

                if (sub_.getConveyor().getAction() == null || sub_.getConveyor().getAction().isDone()) {
                    //
                    // The conveyor has delivered all balls, go to finishing state which
                    // keeps the wheels running while the last ball leaves the robot
                    //
                    shutdown_start_timer_.start() ;

                    state_ = State.FINISHING ;
                }
                break ;            

            case FINISHING:
                if (shutdown_start_timer_.isExpired()) {
                    shooter_action_.stopPlot();
                    sub_.getShooter().setAction(null, true) ;
                    state_ = State.IDLE ;
                    sub_.endPlot(plot_id_);
                    setDone() ;
                }
                break ;
        }

        //
        // Log the current fire action state
        //

        logger.startMessage(MessageType.Debug, fire_action_id_) ;
        logger.add("state", state_.toString()) ;

        if (state_ == State.WAITING) {
            if (has_target_) {
                logger.add("spvalid", shoot_params_valid_) ;
                if (shoot_params_valid_) {
                    logger.add("thood", shoot_params_.hood_) ;
                    logger.add("tvelocity", shoot_params_.v1_) ;
                    logger.add("ahood", sub_.getShooter().getHoodMotor().getPosition()) ;
                    logger.add("am1", sub_.getShooter().getWheelMotor1().getVelocity()) ;
                    logger.add("am2", sub_.getShooter().getWheelMotor2().getVelocity()) ;
                }
                logger.add("shooterready", shooter_ready_) ;
            }
            else {
                logger.add(", no target") ;
            }

            logger.add("dbready", db_ready_) ;
            logger.add("turretready", turret_ready_) ;
        }
        logger.endMessage();

        plot_data_[0] = sub_.getRobot().getTime() - plot_start_ ;
        plot_data_[1] = target_tracker_.getDistance() ;
        plot_data_[2] = (double)go_ ;
        plot_data_[3] = shoot_params_.v1_ ;
        plot_data_[4] = sub_.getShooter().getWheelMotor1().getVelocity() ;
        plot_data_[5] = sub_.getShooter().getWheelMotor2().getVelocity() ;
        sub_.addPlotData(plot_id_, plot_data_) ;
    }

    @Override
    public void cancel() {
        super.cancel();

        sub_.endPlot(plot_id_);

        shooter_action_.stopPlot();

        conveyor_shoot_action_.cancel();
        shooter_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMFireAction";
    }

    private boolean isDBReady() {
        XeroGyro gyro = db_.getGyro() ;

        MessageLogger logger = sub_.getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, fire_action_id_) ;
        logger.add("isDbReady:") ;
        logger.add("lvel", db_.getLeftVelocity()) ;
        logger.add("rvel", db_.getRightVelocity()) ;
        logger.add("gyrox", gyro.getGyroX()) ;
        logger.add("gyroy", gyro.getGyroY()) ;
        logger.add("gyroz", gyro.getGyroZ()) ;
        logger.add("accelx", gyro.getAccelX()) ;
        logger.add("accely", gyro.getAccelY()) ;
        logger.add("accelz", gyro.getAccelZ()) ;
        logger.endMessage();

        if (Math.abs(db_.getLeftVelocity()) > db_velocity_threshold_ || Math.abs(db_.getRightVelocity()) > db_velocity_threshold_)
            return false ;

        if (Math.abs(gyro.getGyroX()) > gyro_threshold_)
            return false ;

        if (Math.abs(gyro.getGyroY()) > gyro_threshold_)
            return false ;

        if (Math.abs(gyro.getGyroX()) > gyro_threshold_)
            return false ;        
            
        if (Math.abs(gyro.getAccelX()) > accel_threshold_)
            return false ;  

        if (Math.abs(gyro.getAccelY()) > accel_threshold_)
            return false ;  
            
        if (Math.abs(gyro.getAccelZ()) > accel_threshold_)
            return false ;              

        return true ;
    }

    private boolean isShooterReady() {
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
        double hood, vel ;
       
        if (dist < 40) {
            //
            // If the shooter exceeds a given distance, we are too far for the
            // hood or the shooter wheels.
            //
            ret = false ;
            vel = pwl_velocity_.getValue(45) ;
            hood = pwl_hood_.getValue(45) ;
        }
        else if (dist > 175) {
            ret = false ;
            vel = pwl_velocity_.getValue(200) ;
            hood = pwl_hood_.getValue(200) ;
        }
        else {
            vel = pwl_velocity_.getValue(dist) ;
            hood = pwl_hood_.getValue(dist) ;
        }
        shoot_params_ = new ShootParams(vel, vel, hood) ;
        return ret ;
    }
}
