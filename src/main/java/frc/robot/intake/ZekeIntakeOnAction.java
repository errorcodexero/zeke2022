package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeOnAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double in_speed_;
    private double out_slow_;
    private double out_fast_;
    private double motor_left_begin_time_;
    private double motor_right_begin_time_;
    private int blocked_count_;
    private double eject_duration_;
    private double blocked_duration_;
    private double left_move_duration_;
    private MovementState motor_left_state_;
    private MovementState motor_right_state_;

    private enum MovementState {
        STOPPED,
        CONVEYOR,
        EJECT,
        UNBLOCK
    }

    public ZekeIntakeOnAction(ZekeIntakeSubsystem subsystem)
            throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        in_speed_ = subsystem_.getSettingsValue("intake-on:in_speed").getDouble();
        out_slow_ = subsystem_.getSettingsValue("intake-on:out_slow").getDouble();
        out_fast_ = subsystem_.getSettingsValue("intake-on:out_fast").getDouble();
        blocked_count_ = subsystem_.getSettingsValue("intake-on:blocked_count").getInteger();
        blocked_duration_ = subsystem_.getSettingsValue("intake-on:blocked_duration").getDouble();
        left_move_duration_ = subsystem_.getSettingsValue("intake-on:move_left_duration").getDouble();
        eject_duration_ = subsystem_.getSettingsValue("intake-on:eject_duration").getDouble();
    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.deployIntake();
        subsystem_.setLeftCollectorPower(in_speed_);
        subsystem_.setRightCollectorPower(in_speed_);
        motor_left_state_ = MovementState.CONVEYOR;
        motor_right_state_ = MovementState.CONVEYOR;
    }

    @Override
    public void run() throws Exception {
        super.run();

        // Butch: added for debugging
        MovementState prevleft = motor_left_state_ ;
        MovementState prevright = motor_right_state_ ;

        // Butch: This is called automatically from the framework.  You don't need this
        // subsystem_.computeMyState();

        CargoType left_color_ = subsystem_.getLeftBallColor();
        CargoType right_color_ = subsystem_.getRightBallColor();

        if (left_color_ == CargoType.Same && right_color_ == CargoType.Same
                && subsystem_.getLeftCount() >= blocked_count_ && subsystem_.getRightCount() >= blocked_count_
                && motor_left_state_ == MovementState.CONVEYOR && motor_right_state_ == MovementState.CONVEYOR) {
            motor_left_state_ = MovementState.UNBLOCK;
            motor_right_state_ = MovementState.UNBLOCK;
            motor_left_begin_time_ = subsystem_.getRobot().getTime();
            motor_right_begin_time_ = subsystem_.getRobot().getTime();

            subsystem_.setLeftCollectorPower(out_slow_);

        } else if (motor_left_state_ == MovementState.UNBLOCK && motor_right_state_ == MovementState.UNBLOCK) {
            if (subsystem_.getRobot().getTime() - motor_right_begin_time_ > blocked_duration_
                    && subsystem_.getRightBallColor() != CargoType.Same) {
                motor_right_state_ = MovementState.CONVEYOR;
                motor_left_state_ = MovementState.CONVEYOR;
                subsystem_.setLeftCollectorPower(in_speed_);
            } else if (subsystem_.getRobot().getTime() - motor_left_begin_time_ > left_move_duration_) {
                subsystem_.setLeftCollectorPower(0);
            }
        } else {
            if (motor_left_state_ == MovementState.CONVEYOR && subsystem_.getLeftBallColor() == CargoType.Opposite) {
                motor_left_state_ = MovementState.EJECT;
                subsystem_.setLeftCollectorPower(out_fast_);
                motor_left_begin_time_ = subsystem_.getRobot().getTime();
            } else if (motor_left_state_ == MovementState.EJECT
                    && subsystem_.getRobot().getTime() - motor_left_begin_time_ > eject_duration_
                    && subsystem_.getLeftBallColor() != CargoType.Opposite) {
                motor_left_state_ = MovementState.CONVEYOR;
                subsystem_.setLeftCollectorPower(in_speed_);
            }
            if (motor_right_state_ == MovementState.CONVEYOR && subsystem_.getRightBallColor() == CargoType.Opposite) {
                motor_right_state_ = MovementState.EJECT;
                subsystem_.setRightCollectorPower(out_fast_);
                motor_right_begin_time_ = subsystem_.getRobot().getTime();
            } else if (motor_right_state_ == MovementState.EJECT
                    && subsystem_.getRobot().getTime() - motor_right_begin_time_ > eject_duration_
                    && subsystem_.getRightBallColor() != CargoType.Opposite) {
                motor_right_state_ = MovementState.CONVEYOR;
                subsystem_.setRightCollectorPower(in_speed_);
            }

        }

        if (prevleft != motor_left_state_ || prevright != motor_right_state_) {
            MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
            logger.add("ZekeIntakeOnAction:").add("time", subsystem_.getRobot().getTime()) ;
            
            if (prevleft != motor_left_state_) {
                logger.add(" left ").add(prevleft.toString()).add(" --> ").add(motor_left_state_.toString()) ;
            }

            if (prevright != motor_right_state_) {
                logger.add(" right ").add(prevright.toString()).add(" --> ").add(motor_right_state_.toString()) ;
            }

            logger.endMessage();
        }

    }

    @Override
    public void cancel() {
        super.cancel();

        try {
            subsystem_.setRightCollectorPower(0);
            subsystem_.setLeftCollectorPower(0);
        } catch(Exception ignored) {}
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOnAction";
    }

}
