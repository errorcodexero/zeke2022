package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.intake.ZekeIntakeSubsystem.MovementState;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeOnAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double in_speed_;
    private double out_slow_;
    private double out_fast_;
    private double stopped_;
    
    public ZekeIntakeOnAction(ZekeIntakeSubsystem subsystem)
            throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        in_speed_ = subsystem_.getSettingsValue("hw:collector:in_speed").getDouble();
        out_slow_ = subsystem_.getSettingsValue("hw:collector:out_slow").getDouble();
        out_fast_ = subsystem_.getSettingsValue("hw:collector:out_fast").getDouble();
        stopped_ = subsystem_.getSettingsValue("hw:collector:stopped").getDouble();

    }

    private void clearIntake()
            throws BadParameterTypeException, MissingParameterException,
            BadMotorRequestException, MotorRequestFailedException,
            InterruptedException {
        double collector_motor_left_power_ = subsystem_.getSettingsValue("hw:collector:motor-left:power").getDouble();
        double collector_motor_right_power_ = subsystem_.getSettingsValue("hw:collector:motor-right:power").getDouble();

        if (!subsystem_.isIntakeBlocked()) {
            subsystem_.setCollectorPower(collector_motor_left_power_, collector_motor_right_power_);
            return;
        }

        if (subsystem_.getRightBallColor() == CargoType.Opposite &&
                subsystem_.getLeftBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(-collector_motor_left_power_, -collector_motor_right_power_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.Opposite &&
                subsystem_.getRightBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(collector_motor_left_power_, -collector_motor_right_power_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.Opposite &&
                subsystem_.getRightBallColor() == CargoType.Same) {
            subsystem_.setCollectorPower(-collector_motor_left_power_, collector_motor_right_power_);
        }
        if (subsystem_.getLeftBallColor() == CargoType.Opposite &&
                subsystem_.getRightBallColor() == CargoType.None) {
            subsystem_.setCollectorPower(-collector_motor_left_power_, collector_motor_right_power_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.None &&
                subsystem_.getRightBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(collector_motor_left_power_, -collector_motor_right_power_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.Same &&
                subsystem_.getRightBallColor() == CargoType.Same) {
            subsystem_.setCollectorPower(-0.1, collector_motor_right_power_);
            subsystem_.getRobot().wait(15L);
            subsystem_.setCollectorPower(collector_motor_left_power_, collector_motor_right_power_);
        }
    }

    @Override
    public void start() throws Exception {
        super.start();
        subsystem_.deployIntake();
        subsystem_.setCollectorPower(in_speed_, in_speed_);
        subsystem_.motor_left_state_ = MovementState.CONVEYOR;
        subsystem_.motor_right_state_ = MovementState.CONVEYOR;
    }

    @Override
    public void run() throws Exception {
        super.run();

        CargoType left_color_ = subsystem_.getLeftBallColor();
        CargoType right_color_ = subsystem_.getRightBallColor();
        if (left_color_ == CargoType.Opposite) {
            subsystem_.motor_left_state_ = MovementState.EJECT;
            subsystem_.setCollectorPower(out_fast_, in_speed_);
        }

        if (!subsystem_.isIntakeBlocked()) {
            subsystem_.setCollectorPower(in_speed_, in_speed_);
        }

        if (subsystem_.getRightBallColor() == CargoType.Opposite &&
                subsystem_.getLeftBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(out_fast_, out_fast_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.Same &&
                subsystem_.getRightBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(in_speed_, out_fast_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.Opposite &&
                subsystem_.getRightBallColor() == CargoType.Same) {
            subsystem_.setCollectorPower(out_fast_, in_speed_);
        }
        if (subsystem_.getLeftBallColor() == CargoType.Opposite &&
                subsystem_.getRightBallColor() == CargoType.None) {
            subsystem_.setCollectorPower(out_fast_, in_speed_);
        }

        if (subsystem_.getLeftBallColor() == CargoType.None &&
                subsystem_.getRightBallColor() == CargoType.Opposite) {
            subsystem_.setCollectorPower(in_speed_, out_fast_);
        }

        if (subsystem_.isIntakeBlocked()) {
                    int motor_left_state_count_ = subsystem_.motor_left_state_count_;
                    int motor_right_state_count_ = subsystem_.motor_right_state_count_;
                    if (motor_left_state_count_ >= 10 && motor_right_state_count_ >= 10) {
                        subsystem_.motor_left_state_ = MovementState.UNBLOCK;
                        subsystem_.motor_right_state_ = MovementState.UNBLOCK;
                        subsystem_.setCollectorPower(out_slow_, in_speed_);
                        while (subsystem_.getRightBallColor() != CargoType.None) {}
                        subsystem_.setCollectorPower(in_speed_, in_speed_);
                        subsystem_.motor_left_state_ = MovementState.CONVEYOR;
                        subsystem_.motor_right_state_ = MovementState.CONVEYOR;
                        subsystem_.motor_left_state_count_ = 0;
                        subsystem_.motor_right_state_count_ = 0;
                    }
                    subsystem_.motor_left_state_ = MovementState.STOPPED;
                    subsystem_.motor_right_state_ = MovementState.STOPPED;
                    subsystem_.motor_right_state_count_++;
                    subsystem_.motor_left_state_count_++;
        }

    }

    @Override
    public void cancel() {
        super.cancel();

        try {
            subsystem_.retractIntake();
            subsystem_.setCollectorPower(0.0, 0.0);
        } catch (Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOnAction";
    }

}