package frc.robot.zeke_color_sensor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import org.xero1425.base.Subsystem;
import org.xero1425.base.misc.ColorSensorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class ZekeColorSensor extends ColorSensorSubsystem {    
    //
    // TODO - update these based on the readings seen when 
    //        sampling a red and blue ball
    //
    final private Color red_ ;
    final private Color blue_ ;
    final private Color none_  ;

    private ColorMatch matcher_  ;
    private int left_intake_ ;
    private int right_intake_ ;
    private int conveyor_ ;

    final private Alliance alliance_ ;

    public ZekeColorSensor(Subsystem parent, I2C.Port port) throws BadParameterTypeException, MissingParameterException {
        super(parent, "zeke-color-sensor", port) ;

        left_intake_ = getSettingsValue("intake-left-index").getInteger() ;
        right_intake_ = getSettingsValue("intake-right-index").getInteger() ;
        conveyor_ = getSettingsValue("conveyor-index").getInteger() ;

        double r, g, b ;

        r = getSettingsValue("colors:red-ball:r").getDouble() ;
        g = getSettingsValue("colors:red-ball:g").getDouble() ;
        b = getSettingsValue("colors:red-ball:b").getDouble() ;
        red_ = new Color(r, g, b) ;

        r = getSettingsValue("colors:blue-ball:r").getDouble() ;
        g = getSettingsValue("colors:blue-ball:g").getDouble() ;
        b = getSettingsValue("colors:blue-ball:b").getDouble() ;
        blue_ = new Color(r, g, b) ;

        r = getSettingsValue("colors:no-ball:r").getDouble() ;
        g = getSettingsValue("colors:no-ball:g").getDouble() ;
        b = getSettingsValue("colors:no-ball:b").getDouble() ;
        none_ = new Color(r, g, b) ;

        matcher_ = new ColorMatch() ;
        matcher_.addColorMatch(red_);
        matcher_.addColorMatch(blue_);
        matcher_.addColorMatch(none_);

        alliance_ = DriverStation.getAlliance() ;
    }

    public enum CargoType {
        None,
        Same,
        Opposite,
    }

    private enum CargoColor {
        Red,
        Blue,
        None
    }

    public int getIntakeLeftIndex() {
        return left_intake_ ;
    }

    public int getIntakeRightIndex() {
        return right_intake_ ;
    }

    public int getConveyorIndex() {
        return conveyor_ ;
    }

    @Override
    public void computeMyState() {
        super.computeMyState();

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Cargo Color:") ;
        for (int i = 0 ; i < count() ; i++) {
            CargoType cargo = getCargoType(i) ;
            logger.add(" ").add(cargo.toString()) ;
        }
        logger.endMessage();
    }

    public CargoType getCargoType(int which) {
        CargoType type = CargoType.None ;

        Color c = getColor(which) ;
        CargoColor cc = colorToCargoColor(c) ;

        if (cc == CargoColor.Red) {
            type = (alliance_ == Alliance.Red) ? CargoType.Same  : CargoType.Opposite ;
        }
        else if (cc == CargoColor.Blue) {
            type = (alliance_ == Alliance.Blue) ? CargoType.Same  : CargoType.Opposite ;
        }

        return type ;
    }

    private CargoColor colorToCargoColor(Color c) {

        ColorMatchResult result = matcher_.matchClosestColor(c) ;
        if (result.color == red_)
            return CargoColor.Red ;
        else if (result.color == blue_)
            return CargoColor.Blue ;
            
        return CargoColor.None ;
    }
}
