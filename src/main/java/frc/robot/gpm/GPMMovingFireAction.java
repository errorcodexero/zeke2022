package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.base.utils.TargetTracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.conveyor.ConveyorShootAction;
import frc.robot.shooter.SetShooterAction;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;

public class GPMMovingFireAction extends Action { 
    private final String LoggerName = "fire-action" ;

    private GPMSubsystem sub_;
    private TargetTrackerSubsystem target_tracker_ ;
    private TankDriveSubsystem db_ ;
    private TurretSubsystem turret_ ;

    private ShootParams shoot_params_ ;
    private double conveyor_shoot_latency_ ;

    private ConveyorShootAction conveyor_shoot_action_ ;
    private SetShooterAction shooter_action_ ;
    private MotorEncoderTrackPositionAction turret_action_ ;

    // Butch: These are just variables, nothing special in the code, lets follow our standard convention
    private double db_velocity_threshold_ ;
    private double shooter_velocity_threshold_ ;
    private double hood_position_threshold_ ;

    private Translation2d target_pos_ ;
    private TargetTracker tracker_ ;

    private int logger_id_ ;
    
    public GPMMovingFireAction(GPMSubsystem sub, TankDriveSubsystem db, TurretSubsystem turret) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        db_ = db ;
        turret_ = turret ;

        logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem(LoggerName) ;

        double index ;
        index = sub_.getSettingsValue("fire-action:db_vel_threshold").getDouble() ;
        db_velocity_threshold_ = index;

        index = sub_.getSettingsValue("fire-action:shooter_vel_threshold").getDouble() ;
        shooter_velocity_threshold_ = index;

        index = sub_.getSettingsValue("fire-action:hood_pos_threshold").getDouble() ;
        hood_position_threshold_ = index;

        conveyor_shoot_latency_ = sub_.getSettingsValue("fire-action:shoot-latency").getDouble() ;
        target_pos_ = new Translation2d(27.0 * 12.0, 27.0 * 12.0) ;
        tracker_ = new TargetTracker(target_pos_) ;

        // figure out intake and shooter doubles -> get from params
        conveyor_shoot_action_ = new ConveyorShootAction(sub_.getConveyor()) ; 
        
        // figure out what w1, w2, and hood default vals are from -> get from params file        
        shoot_params_ = new ShootParams(0.0, 0.0, 0.0) ;

        shooter_action_ = new SetShooterAction(sub_.getShooter(), shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        sub_.getShooter().setAction(shooter_action_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        boolean shooterReady, dbready, turretready ;

        computeMoveValues() ;

        shooterReady = isShooterReady() ;
        dbready = Math.abs(db_.getVelocity()) < db_velocity_threshold_ ;
        turretready = isTurretReady() ;

        // if the
        //  * shooter is up to speed
        //  * db is stopped
        //  * target tracker sees the target
        //  * turret is aimed & ready to fire
        // then, let the conveyor push cargo into the shooter
        if (shooterReady && dbready && turretready)
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

    private double getTotalLatency() {
        return target_tracker_.getLatency() + conveyor_shoot_latency_ ;
    }

    private void computeMoveValues() throws BadMotorRequestException, MotorRequestFailedException {

        // Current position of the robot
        Pose2d robot = db_.getPose() ;

        // Current velocity of the robot
        Vector2d dbvel = db_.getVelocityVector() ;

        // Note, the heading of the robot and the velocity vectory should point the same way.  There
        // may be some difference because the velocity vector uses the ram gyro angle value and not the
        // averaged value.

        // The total latency of the target tracker (e.g. limelight) and the shooter
        double latency = getTotalLatency() ;
    
        // Compute the position of the robot when the shot would actually leave the robot
        Transform2d t = new Transform2d(new Translation2d(latency * dbvel.x, latency * dbvel.y), new Rotation2d()) ;
        Pose2d newpos = robot.transformBy(t) ;

        // Update the turret position based on our future position
        tracker_.getRelativeTargetAngle(robot) ;
        turret_action_.setTarget(tracker_.getRelativeTargetAngle(robot));

        // Update the shooter parameters based on our future position
        shoot_params_ = calculate(tracker_.getRelativeTargetDistance(newpos)) ;
        shooter_action_.update(shoot_params_.v1_, shoot_params_.v2_, shoot_params_.hood_) ;
    }

    private boolean isTurretReady() {
        return turret_action_.getError() < 5.0 ;
    }
    
    private boolean isShooterReady() {
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

    public ShootParams calculate(double distance) {

        double v1 = 10.0 * distance ;
        double v2 = 10.0 * distance ;
        double hood = 1 * distance ;

        return new ShootParams(v1, v2, hood) ;
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
