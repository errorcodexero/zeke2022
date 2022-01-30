package frc.models;

import org.xero1425.base.misc.ColorSensorSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorModel extends SimulationModel {
    private static final String ColorPrefix = "color-" ;
    private static final String IRPrefix = "ir-" ;
    private static final String ProximityPrefix = "proximity-" ;

    private byte mux_data_ ;
    private int sensor_handle_ ;
    private int mux_handle_ ;
    private Color [] colors_ ;
    private int [] proximity_ ;
    private int [] irvalues_ ;

    public ColorSensorModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        mux_data_ = 0 ;
    }

    public boolean create() {
        int count ;

        sensor_handle_ = SimDeviceDataJNI.getSimDeviceHandle("REV Color Sensor V3[1,82]") ;
        mux_handle_ = SimDeviceDataJNI.getSimDeviceHandle(ColorSensorSubsystem.I2CMuxSimDevName) ;
        
        try {
            count = getIntProperty("count") ;
        }
        catch(Exception ex) {
            return false ;
        }

        colors_ = new Color[count] ;
        proximity_ = new int[count] ;
        irvalues_ = new int[count] ;
        for(int i = 0 ; i < count ; i++) {
            colors_[i] = Color.kBlack ;
            proximity_[i] = 0 ;
            irvalues_[i] = 0 ;
        }

        setCreated();
        
        return true ;
    }

    public boolean processEvent(String name, SettingsValue value) {
        if (name.startsWith(ColorPrefix)) {
            int which = Integer.parseInt(name.substring(ColorPrefix.length())) ;
            if (value.isString()) {
                String [] comps  = null ;

                try {
                    comps = value.getString().split(",") ;
                }
                catch(Exception ex) {
                    // Will not happen
                }

                if (comps == null || comps.length != 3) {
                    MessageLogger logger = getEngine().getMessageLogger() ;
                    logger.startMessage(MessageType.Error);
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(" the parameter ").addQuoted(name) ;
                    logger.add(" is an RGB value, three decimal number between 0.0 and 1.0 and seperated by commas") ;
                    logger.endMessage();

                    return true ;
                }

                double r = Double.parseDouble(comps[0]) ;
                double g = Double.parseDouble(comps[1]) ;
                double b = Double.parseDouble(comps[2]) ;

                if (r < 0.0 || r > 1.0 || g < 0.0 || g > 1.0 || b < 0.0 || b > 1.0) {
                    MessageLogger logger = getEngine().getMessageLogger() ;
                    logger.startMessage(MessageType.Error);
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(" the parameter ").addQuoted(name) ;
                    logger.add(" is an RGB value, three decimal number between 0.0 and 1.0 and seperated by commas") ;
                    logger.endMessage();

                    return true ;
                }

                colors_[which] = new Color(r, g, b) ;
            }
        }
        else if (name.startsWith(ProximityPrefix)) {
            int which = Integer.parseInt(name.substring(ProximityPrefix.length())) ;
            if (value.isInteger()) {
                try {
                    proximity_[which] = value.getInteger() ;
                }
                catch(Exception ex) {

                }
            }
        }
        else if (name.startsWith(IRPrefix)) {
            int which = Integer.parseInt(name.substring(IRPrefix.length())) ;
            if (value.isInteger()) {
                try {
                    irvalues_[which] = value.getInteger() ;
                }
                catch(Exception ex) {
                    
                }
            }
        }
        return true;
    }

    public void run(double dt) {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(mux_handle_, ColorSensorSubsystem.I2CMuxSimValueName) ;
        HALValue v = SimDeviceJNI.getSimValue(vhandle) ;

        if (v.getType() == HALValue.kInt) {
            byte b = (byte)v.getLong() ;
            if (b != mux_data_) {
                mux_data_ = b ;
                int which = bitmaskToWhich(mux_data_) ;

                // System.out.println("Color sensor model running, which " + which) ;

                if (which != -1) {
                    //System.out.print("Setting color sensor values for index " + which) ;
                    //System.out.print(" RGB: " + colors_[which].red + ", " + colors_[which].green + ", " + colors_[which].blue) ;
                    //System.out.println(", prox " + proximity_[which] + ", ir " + irvalues_[which]) ;

                    int handle ;
                    
                    handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, "Red") ;
                    SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(colors_[which].red)) ;

                    handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, "Green") ;
                    SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(colors_[which].red)) ;

                    handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, "Blue") ;
                    SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(colors_[which].red)) ;

                    handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, "Proximity") ;
                    SimDeviceJNI.setSimValue(handle, HALValue.makeInt(proximity_[which])) ;

                    handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, "IR") ;
                    SimDeviceJNI.setSimValue(handle, HALValue.makeInt(irvalues_[which])) ;
                }
            }
        }
    }

    private int bitmaskToWhich(byte mask) {
        for(int i = 0 ; i < 8 ; i++) {
            if ((mask & (1 << i)) != 0)
                return i ;
        }

        return -1 ;
    }
}
