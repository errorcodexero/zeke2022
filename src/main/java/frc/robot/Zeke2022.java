package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SimArgs;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.automodes.ZekeAutoController;
import frc.robot.zekesubsystem.ZekeSubsystem ;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Zeke2022 extends XeroRobot {
  public Zeke2022() {
    super(0.02) ;
  }
  
  public String getName() {
    return "zeke2022";
  }

  public AutoController createAutoController() throws MissingParameterException, BadParameterTypeException {
    return new ZekeAutoController(this);
  }
  
  protected void hardwareInit() throws Exception {
    enablePneumatics() ;
    Compressor comp = getCompressor() ;
    comp.enableAnalog(110, 120);

    ZekeSubsystem robotsub = new ZekeSubsystem(this) ;
    setRobotSubsystem(robotsub) ;
  }

  public String getSimulationFileName() {
    String ret = SimArgs.InputFileName;
    if (ret != null)
      return ret;

    return "intake-one-same";
  }

  protected void addRobotSimulationModels() {
    ModelFactory factory = SimulationEngine.getInstance().getModelFactory() ;
    factory.registerModel("climber", "frc.models.ClimberModel");    
    factory.registerModel("color-sensor-model", "frc.models.ColorSensorModel") ;
  }
}