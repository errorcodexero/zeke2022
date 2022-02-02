package frc.robot.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.SettingsValue.SettingsType;

import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeSubsystem extends Subsystem {
  public static final String SubsystemName = "intake";

  private MotorController collector_left_;
  private MotorController collector_right_;
  private XeroSolenoid solenoid_;
  private ZekeColorSensor color_sensor_;
  public int motor_left_state_count_;
  public int motor_right_state_count_;
  public MovementState motor_left_state_;
  public MovementState motor_right_state_;
  private double left_motor_power_;
  private double right_motor_power_;

  public enum MotorSpeed {
    IN,
    OUTSLOW,
    OUTFAST,
    STOPPED
  }

  public enum MovementState {
    STOPPED,
    CONVEYOR,
    EJECT,
    UNBLOCK
  }

  public ZekeIntakeSubsystem(Subsystem parent, ZekeColorSensor sensor) throws Exception {
    super(parent, SubsystemName);

    collector_left_ = getRobot().getMotorFactory().createMotor(
        "intake-collector-left", "subsystems:intake:hw:collector:motor-left");
    collector_right_ = getRobot().getMotorFactory().createMotor(
        "intake-collecor-right", "subsystems:intake:hw:collector:motor-right");

    solenoid_ = new XeroSolenoid(this, "deploy");
    color_sensor_ = sensor ;
    motor_left_state_count_ = 0;
    motor_right_state_count_= 0;
    
  }

  public void setCollectorPower(double pa, double pb)
      throws BadMotorRequestException, MotorRequestFailedException {
    collector_left_.set(pa);
    collector_right_.set(pb);
    left_motor_power_ = pa;
    right_motor_power_ = pb;
  }

  
  public void deployIntake() {
    solenoid_.set(true);
  }
  public void retractIntake() {
    solenoid_.set(false);
  }

  public CargoType getLeftBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeLeftIndex()); }
  public CargoType getRightBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeRightIndex()); }

  public int getLeftCount() {
    return motor_left_state_count_;

  }
  public int getRightCount() {
    return motor_right_state_count_;
  }
  public boolean isIntakeBlocked() {
    return motor_left_state_ == MovementState.STOPPED && motor_right_state_ == MovementState.STOPPED;
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
