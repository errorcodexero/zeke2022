package frc.robot.gpm.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;;

public class ZekeIntakeSubsystem extends Subsystem {
  public static final String SubsystemName = "intake";

  private MotorController collector_a_;
  private MotorController collector_b_;
  private XeroSolenoid solenoid_;
  private ZekeColorSensor color_sensor_;

  public ZekeIntakeSubsystem(Subsystem parent) throws Exception {
    super(parent, SubsystemName);

    collector_a_ = getRobot().getMotorFactory().createMotor(
        "intake-collector-a", "subsystems:intake:hw:collector:motor-a");
    collector_b_ = getRobot().getMotorFactory().createMotor(
        "intake-collecor-b", "subsystems:intake:hw:collector:motor-b");

    solenoid_ = new XeroSolenoid(this, "deploy");
    color_sensor_ = new ZekeColorSensor(parent, I2C.Port.kMXP , 1);
    
  }

  public void setCollectorPower(double pa, double pb)
      throws BadMotorRequestException, MotorRequestFailedException {
    collector_a_.set(pa);
    collector_b_.set(pb);
  }

  
  public void deploySolenoid() {
    solenoid_.set(true);
  }
  public void retractSolenoid() {
    solenoid_.set(false);
  }

  public CargoType getLeftBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeLeftIndex()); }
  public CargoType getRightBallColor() { return color_sensor_.getCargoType(color_sensor_.getIntakeRightIndex()); }
  public boolean isIntakeBlocked() {
    return getLeftBallColor() != CargoType.None &&
        getRightBallColor() != CargoType.None;
  }

  public void clearIntake()
      throws BadParameterTypeException, MissingParameterException,
             BadMotorRequestException, MotorRequestFailedException,
             InterruptedException {
    double collector_motor_a_power_ =
        getSettingsValue("hw:collector:motor-a:power").getDouble();
    double collector_motor_b_power_ =
        getSettingsValue("hw:collector:motor-b:power").getDouble();

    if (!isIntakeBlocked()) {
      setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
      return;
    }

    if (getRightBallColor() == CargoType.Opposite&&
        getLeftBallColor() == CargoType.Opposite) {
      setCollectorPower(-collector_motor_a_power_, -collector_motor_b_power_);
    }

    if (getLeftBallColor() == CargoType.Opposite &&
        getRightBallColor() == CargoType.Opposite ) {
      setCollectorPower(collector_motor_a_power_, -collector_motor_b_power_);
    }

    if (getLeftBallColor() == CargoType.Opposite &&
        getRightBallColor() == CargoType.Same) {
      setCollectorPower(-collector_motor_a_power_, collector_motor_b_power_);
    }
    if (getLeftBallColor() == CargoType.Opposite &&
        getRightBallColor() == CargoType.None) {
      setCollectorPower(-collector_motor_a_power_, collector_motor_b_power_);
    }

    if (getLeftBallColor() == CargoType.None&&
        getRightBallColor() == CargoType.Opposite) {
      setCollectorPower(collector_motor_a_power_, -collector_motor_b_power_);
    }

    if (getLeftBallColor() == CargoType.Same &&
        getRightBallColor() == CargoType.Same) {
      setCollectorPower(-0.1, collector_motor_b_power_);
      getRobot().wait(15L);
      setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
    }
  }

  @Override
  public void postHWInit() {
    retractSolenoid();
  }
}
