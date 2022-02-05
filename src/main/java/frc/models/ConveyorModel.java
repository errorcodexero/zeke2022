package frc.models;

import edu.wpi.first.hal.simulation.DIODataJNI;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsValue;

public class ConveyorModel extends SimulationModel {
    
    private int intake_sensor_io_ ;
    private int chimney_sensor_io_ ;
    private int shooter_sensor_io_ ;
    private int exit_sensor_io_ ;

    public ConveyorModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
    }

    @Override
    public void endCycle() {
    }

    public boolean create() {
        try {
            intake_sensor_io_ = getProperty("intake-sensor").getInteger() ;
            DIODataJNI.setIsInput(intake_sensor_io_, true) ;
            DIODataJNI.setValue(intake_sensor_io_, true) ;

            chimney_sensor_io_ = getProperty("chimney-sensor").getInteger() ;
            DIODataJNI.setIsInput(chimney_sensor_io_, true) ;
            DIODataJNI.setValue(chimney_sensor_io_, true) ;

            shooter_sensor_io_ = getProperty("shooter-sensor").getInteger() ;
            DIODataJNI.setIsInput(shooter_sensor_io_, true) ;
            DIODataJNI.setValue(shooter_sensor_io_, true) ;

            exit_sensor_io_ = getProperty("exit-sensor").getInteger() ;
            DIODataJNI.setIsInput(exit_sensor_io_, true) ;
            DIODataJNI.setValue(exit_sensor_io_, true) ;
        }
        catch(BadParameterTypeException ex) {
            return false ;
        }

        setCreated();
        return true ;
    }

    public boolean processEvent(String name, SettingsValue value) {
        try {
            if (name.equals("intake")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(intake_sensor_io_, value.getBoolean()) ;
                }
            }
            else if (name.equals("chimney")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(chimney_sensor_io_, value.getBoolean()) ;
                }
            }
            else if (name.equals("shooter")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(shooter_sensor_io_, value.getBoolean()) ;
                }
            }            
            else if (name.equals("exit")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(exit_sensor_io_, value.getBoolean()) ;
                }
            } 
        }
        catch(BadParameterTypeException ex) {
            //
            // Should never happen, we check the types before getting the value
            //
        }
        return true ;
    }

    public void run(double dt) {
    }
}