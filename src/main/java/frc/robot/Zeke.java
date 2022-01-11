package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Zeke extends XeroRobot {
  public Zeke() {
    super(0.02) ;
  }
  
  public String Zeke() {
    return "zeke2022";
  }

  public AutoController createAutoController() {
    return null ;
  }
  
  protected void hardwareInit() throws Exception {
  }
}