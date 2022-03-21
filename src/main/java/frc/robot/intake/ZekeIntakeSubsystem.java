package frc.robot.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.SettingsValue;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeSubsystem extends Subsystem {
  public static final String SubsystemName = "intake";

  private MotorController collector_left_;
  private MotorController collector_right_;
  private XeroSolenoid solenoid_;
  private ZekeColorSensor color_sensor_;
  private double left_motor_power_;
  private double right_motor_power_;
  private CargoType left_intake_color_;
  private CargoType right_intake_color_;
  private int left_count_;
  private int right_count_;

  public ZekeIntakeSubsystem(Subsystem parent, ZekeColorSensor sensor) throws Exception {
    super(parent, SubsystemName);

    double ramprate = getSettingsValue("ramp-rate").getDouble() ;

    collector_left_ = getRobot().getMotorFactory().createMotor(
        "intake-collector-left", "subsystems:intake:hw:collector:motor-left");
    collector_right_ = getRobot().getMotorFactory().createMotor(
        "intake-collecor-right", "subsystems:intake:hw:collector:motor-right");

    collector_left_.setOpenLoopRampRate(ramprate) ;
    collector_right_.setOpenLoopRampRate(ramprate) ;

    solenoid_ = new XeroSolenoid(this, "deploy");
    color_sensor_ = sensor;
    left_intake_color_ = CargoType.None;
    right_intake_color_ = CargoType.None;
    left_count_ = 0;
    right_count_ = 0;
  }

  public void setLeftCollectorPower(double power) throws BadMotorRequestException, MotorRequestFailedException {
    collector_left_.set(power);
    left_motor_power_ = power;
  }

  public void setRightCollectorPower(double power) throws BadMotorRequestException, MotorRequestFailedException {
    collector_right_.set(power);
    right_motor_power_ = power;
  }

  @Override
  public void computeMyState() {
    CargoType left = color_sensor_.getCargoType(color_sensor_.getIntakeLeftIndex());
    CargoType right = color_sensor_.getCargoType(color_sensor_.getIntakeRightIndex());
    if (right == right_intake_color_) {
      right_count_++;
    } else {
      right_intake_color_ = right;
      right_count_ = 1;
    }
    if (left == left_intake_color_) {
      left_count_++;
    } else {
      left_intake_color_ = left;
      left_count_ = 1;
    }
  }

  public void deployIntake() {
    solenoid_.set(true);
  }

  public void retractIntake() {
    solenoid_.set(false);
  }

  public CargoType getLeftBallColor() {
    return left_intake_color_;
  }

  public CargoType getRightBallColor() {
    return right_intake_color_;
  }

  public int getLeftCount() {
    return left_count_;
  }

  public int getRightCount() {
    return right_count_;
  }

  @Override
  public SettingsValue getProperty(String name) {
    SettingsValue v = null;
    if (name.equals("left-power")) {
      v = new SettingsValue(left_motor_power_);
    }
    if (name.equals("right-power")) {
      v = new SettingsValue(right_motor_power_);
    }

    return v;

  }

  @Override
  public void postHWInit() {
    retractIntake();
  }
}
