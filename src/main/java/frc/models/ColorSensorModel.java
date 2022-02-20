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

public class ColorSensorModel extends SimulationModel {
    private static final String ColorPrefix = "color-" ;
    private static final String IRPrefix = "ir-" ;
    private static final String ProximityPrefix = "proximity-" ;

    private int sensor_handle_ ;

    public ColorSensorModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
    }

    public boolean create() {
        sensor_handle_ = SimDeviceDataJNI.getSimDeviceHandle(ColorSensorSubsystem.ColorSensorMuxSimDevName) ;
        setCreated();
        
        return true ;
    }

    public boolean processEvent(String name, SettingsValue value) {
        int handle ;

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

                handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, ColorSensorSubsystem.ColorSensorSimRedValueName + which) ;
                SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(r)) ;
                handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, ColorSensorSubsystem.ColorSensorSimGreenValueName + which) ;
                SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(g)) ;
                handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, ColorSensorSubsystem.ColorSensorSimBlueValueName + which) ;
                SimDeviceJNI.setSimValue(handle, HALValue.makeDouble(b)) ;

            }
        }
        else if (name.startsWith(ProximityPrefix)) {
            int v  = 0 ;
            int which = Integer.parseInt(name.substring(ProximityPrefix.length())) ;
            if (value.isInteger()) {
                try {
                   v  = value.getInteger() ;
                }
                catch(Exception ex) {

                }
            }

            handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, ColorSensorSubsystem.ColorSensorSimProximityValueName + which) ;
            SimDeviceJNI.setSimValue(handle, HALValue.makeInt(v)) ;
        }
        else if (name.startsWith(IRPrefix)) {
            int v = 0 ;
            int which = Integer.parseInt(name.substring(IRPrefix.length())) ;
            if (value.isInteger()) {
                try {
                   v  = value.getInteger() ;
                }
                catch(Exception ex) {

                }
            }

            handle = SimDeviceDataJNI.getSimValueHandle(sensor_handle_, ColorSensorSubsystem.ColorSensorSimIRValueName + which) ;
            SimDeviceJNI.setSimValue(handle, HALValue.makeInt(v)) ;
        }
        return true;
    }

    public void run(double dt) {
    }
}
