package org.xero1425.base.misc;

import com.revrobotics.ColorSensorV3;

import org.xero1425.base.Subsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSubsystem extends Subsystem {
    static public final String I2CMuxSimDevName = "i2c-mux" ;
    static public final String I2CMuxSimValueName = "i2c-mux-value" ;

    private I2C.Port port_ ;
    private int which_ ;
    private I2C muxdev_ ;
    private int muxaddr_ ;
    private byte [] data_ = new byte[1] ;
    private byte sample_ ;
    private int count_ ;
    private ColorSensorV3 sensor_ ;
    private int[] proximity_ ;
    private int[] ir_ ;
    private Color [] colors_ ;
    private int current_sensor_ ;

    private SimDevice i2c_mux_ ;
    private SimInt i2c_mux_value_ ;

    public ColorSensorSubsystem(Subsystem parent, String name, I2C.Port port) throws BadParameterTypeException, MissingParameterException {
        super(parent, name) ;

        port_ = port ;
        which_ = -1 ;    

        count_ = getSettingsValue("hw:i2cmux:count").getInteger() ;
        muxaddr_ = getSettingsValue("hw:i2cmux:address").getInteger() ;
       
        if (XeroRobot.isSimulation()) {
            i2c_mux_ = SimDevice.create(I2CMuxSimDevName) ;
            i2c_mux_value_ = i2c_mux_.createInt(I2CMuxSimValueName, SimDevice.Direction.kBidir, 0) ;
        } else {
            i2c_mux_ = null ;
            i2c_mux_value_ = null ;
        }

        muxdev_ = new I2C(port_, muxaddr_)  ;
        select(0) ;

        sample_ = 0 ;
        for(int i = 0 ; i < count_ ; i++)
            sample_ |= (byte)(1 << i) ;

        sensor_ = new ColorSensorV3(port_) ;
        colors_ = new Color[count_] ;
        proximity_ = new int[count_] ;
        ir_ = new int[count_] ;

        for(int i = 0 ; i < count_ ; i++)
            init(i) ;

        current_sensor_ = 0 ;
    }

    public int count() {
        return count_ ;
    }

    public Color getColor(int which) {
        return colors_[which] ;
    }

    public int getProximity(int which) {
        return proximity_[which] ;
    }

    public int getIR(int which) {
        return ir_[which] ;
    }

    @Override
    public void computeMyState() {
        
        if ((sample_ & (1 << current_sensor_)) != 0) {
            select(current_sensor_) ;

            colors_[current_sensor_] = sensor_.getColor() ;
            proximity_[current_sensor_] = sensor_.getProximity() ;
            ir_[current_sensor_] = sensor_.getIR() ;
        }

        current_sensor_++ ;
        if (current_sensor_ == count_)
            current_sensor_ = 0 ;
    }

    public void enableSensor(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        sample_ |= (1 << which) ;
    }

    public void disableSensor(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        sample_ &= ~(1 << which) ;
    }

    public boolean isEnabled(int which) throws Exception {
        if (which >= count_) {
            throw new Exception("invalid sensor number in ColorSensorSubsystem.enableSensor(int which)") ;
        }

        return ((sample_ & (1 << which)) != 0) ;
    }

    // Derived class override this for different initialization
    protected ColorSensorV3.ColorSensorResolution getResolution(int which) {
        return ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit ;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.ColorSensorMeasurementRate getMeasurementRate(int which) { 
        return ColorSensorV3.ColorSensorMeasurementRate.kColorRate100ms ;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.GainFactor getGain(int which) {
        return ColorSensorV3.GainFactor.kGain18x ;
    }

    private void init(int which) {
        select(which) ;

        ColorSensorV3.ColorSensorResolution res = getResolution(which) ;
        ColorSensorV3.ColorSensorMeasurementRate rate = getMeasurementRate(which) ;
        ColorSensorV3.GainFactor gain = getGain(which) ;
        sensor_.configureColorSensor(res, rate, gain) ;        
    }

    private void select(int which) {
        data_[0] = (byte)(0x01 << which) ;

        if (i2c_mux_value_ != null) {
            i2c_mux_value_.set(data_[0]) ;
        }
        System.out.println() ;

        if (muxdev_ != null && which != which_) {

            muxdev_.writeBulk(data_);
            which_ = which ;
        }
    }
}
