package frc.robot.zekeoi;

import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.ChangeClampTo;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeManualClimbGamepad extends Gamepad {
    private double max_power_ ;

    public ZekeManualClimbGamepad(OISubsystem oi, int index) throws BadParameterTypeException, MissingParameterException {
        super(oi, "climber", index) ;

        max_power_ = oi.getSettingsValue("manual-climb:max-windmill-power").getDouble() ;

        disable() ;
    }

    @Override
    public void computeState() {
        super.computeState();

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("Running manual climb compute state") ;
        logger.endMessage();
        

        ZekeSubsystem zeke = (ZekeSubsystem)getSubsystem().getRobot().getRobotSubsystem() ;
        ClimberSubsystem climber = zeke.getClimber() ;

        double stick = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;
        double power = (stick  * max_power_) ;
        climber.getWindmillMotor().setPower(power) ;

        if (isLBackButtonPrssed()) {
            climber.changeClamp(WhichClamp.CLAMP_A, ChangeClampTo.OPEN);
        }
        else if (isLTriggerPressed()) {
            climber.changeClamp(WhichClamp.CLAMP_A, ChangeClampTo.CLOSED);
        }
        else if (isRBackButtonPressed()) {
            climber.changeClamp(WhichClamp.CLAMP_B, ChangeClampTo.OPEN);
        }
        else if (isRTriggerPressed()) {
            climber.changeClamp(WhichClamp.CLAMP_B, ChangeClampTo.CLOSED);
        }
    }
}
