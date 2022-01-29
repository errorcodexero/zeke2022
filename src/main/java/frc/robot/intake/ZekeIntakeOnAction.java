package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeOnAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double collector_motor_a_power_;
    private double collector_motor_b_power_;
    private int blocked_count;
    private States intake_state;

    private enum States {
        BLOCKED,
        COLLECT,
        EMPTY,
    }

    public ZekeIntakeOnAction(ZekeIntakeSubsystem subsystem)
            throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        collector_motor_a_power_ = subsystem_.getSettingsValue("hw:collector:motor-a:power").getDouble();
        collector_motor_b_power_ = subsystem_.getSettingsValue("hw:collector:motor-b:power").getDouble();

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
        subsystem_.setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
        blocked_count = 0;
        intake_state = States.EMPTY;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (subsystem_.isIntakeBlocked()) {
            if (blocked_count >= 10) {
                if (intake_state == States.BLOCKED) {
                    clearIntake();
                    intake_state = States.EMPTY;
                    blocked_count = 0;
                }
            } else {
                blocked_count++;
                intake_state = States.BLOCKED;
            }
        } else {
            subsystem_.setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
            if (subsystem_.getLeftBallColor() == CargoType.None && subsystem_.getRightBallColor() == CargoType.None) {
                blocked_count = 0;
                intake_state = States.EMPTY;
            } else {
                if (subsystem_.getLeftBallColor() == CargoType.None
                        || subsystem_.getRightBallColor() == CargoType.None) {
                    intake_state = States.COLLECT;
                }
            }
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