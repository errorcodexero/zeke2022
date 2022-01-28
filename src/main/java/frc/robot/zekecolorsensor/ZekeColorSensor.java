package frc.robot.zekecolorsensor;

import com.revrobotics.ColorMatch;

import org.xero1425.base.Subsystem;
import org.xero1425.base.misc.ColorSensorSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class ZekeColorSensor extends ColorSensorSubsystem {    
    //
    // TODO - update these based on the readings seen when 
    //        sampling a red and blue ball
    //
    final private Color red_ = new Color(1.0, 0.0, 0.0) ;
    final private Color blue_ = new Color(0.0, 0.0, 1.0) ;
    private ColorMatch matcher_  ;
    final private Alliance alliance_ ;

    public ZekeColorSensor(Subsystem parent, I2C.Port port, int muxaddr) {
        super(parent, "ZekeColorSensor", port, muxaddr, 3) ;

        matcher_ = new ColorMatch() ;
        matcher_.addColorMatch(red_);
        matcher_.addColorMatch(blue_);

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

    @Override
    public void computeMyState() {
        super.computeMyState(); 

        for(int i = 0 ;i < count() ; i++) {
            String str = "none" ;
            if (getCargoType(i) == CargoType.Same)
                str = "same" ;
            else if (getCargoType(i) == CargoType.Opposite)
                str = "opposite" ;

            putDashboard("CS-" + i, DisplayType.Verbose, str) ;
        }
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
        //
        // TODO: used the color matcher, the confidence factor, and the proximity factor
        //       to match a blue or red cargo
        //
        return CargoColor.None ;
    }
}
