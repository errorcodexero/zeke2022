package frc.robot.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeSubsystem extends Subsystem {
  public static final String SubsystemName = "intake";

  private MotorController collector_left_;
  private MotorController collector_right_;
  private XeroSolenoid solenoid_;
  private ZekeColorSensor color_sensor_;

  public ZekeIntakeSubsystem(Subsystem parent, ZekeColorSensor sensor) throws Exception {
    super(parent, SubsystemName);

    collector_left_ = getRobot().getMotorFactory().createMotor(
        "intake-collector-left", "subsystems:intake:hw:collector:motor-left");
    collector_right_ = getRobot().getMotorFactory().createMotor(
        "intake-collecor-right", "subsystems:intake:hw:collector:motor-right");

    solenoid_ = new XeroSolenoid(this, "deploy");
    color_sensor_ = sensor ;
    
  }

  public void setCollectorPower(double pa, double pb)
      throws BadMotorRequestException, MotorRequestFailedException {
    collector_left_.set(pa);
    collector_right_.set(pb);
  }

  
  public void deployIntake() {
    solenoid_.set(true);
  }
  public void retractIntake() {
    solenoid_.set(false);
  }

  public CargoType getLeftBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeLeftIndex()); }
  public CargoType getRightBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeRightIndex()); }
  public boolean isIntakeBlocked() {
    return getLeftBallColor() != CargoType.None &&
        getRightBallColor() != CargoType.None;
  }

  

  @Override
  public void postHWInit() {
    retractIntake();
  }
}
