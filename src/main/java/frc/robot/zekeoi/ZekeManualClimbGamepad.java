package frc.robot.zekeoi;

import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeManualClimbGamepad extends Gamepad {
    private double max_power_ ;

    private double current_power_ ;

    public ZekeManualClimbGamepad(OISubsystem oi, int index) throws BadParameterTypeException, MissingParameterException {
        super(oi, "climber", index) ;

        max_power_ = oi.getSettingsValue("manual-climb:max-windmill-power").getDouble() ;

        disable() ;
    }

    @Override
    public void computeState() {
        super.computeState();

        ZekeSubsystem zeke = (ZekeSubsystem)getSubsystem().getRobot().getRobotSubsystem() ;
        ClimberSubsystem climber = zeke.getClimber() ;

        //
        // Read the joystick from the game pad and apply power to the climber
        //
        double stick = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

        //
        // Scale the stick value to the max_power_ value and assign to windmill motor
        //


        if (isLBackButtonPrssed()) {
            // Open A grabbers
        }
        else if (isLTriggerPressed()) {
            // Close A grabbers
        }
        else if (isRBackButtonPressed()) {
            // Open B grabbers
        }
        else if (isRTriggerPressed()) {
            // Close B grabbers
        }
    }
}
