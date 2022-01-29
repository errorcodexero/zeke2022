package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;

import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.simulation.DIODataJNI;

public class ClimberModel extends SimulationModel {
    private SimMotorController motor_ ;
    private SimInt grabber_left_a_ ;
    private SimInt grabber_left_b_ ;
    private SimInt grabber_right_a_ ;
    private SimInt grabber_right_b_ ;

    private int whisker_left_a1_io_ ;
    private boolean whisker_left_a1_value_ ;
    private int whisker_left_a2_io_ ;
    private boolean whisker_left_a2_value_ ;
    private int whisker_right_a1_io_ ;
    private boolean whisker_right_a1_value_ ;
    private int whisker_right_a2_io_ ;
    private boolean whisker_right_a2_value_ ;
    private int whisker_left_b_io_ ;
    private boolean whisker_left_b_value_ ;
    private int whisker_right_b_io_ ;
    private boolean whisker_right_b_value_ ;

    public ClimberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

    }

    @Override
    public boolean create()  {
        return true ;
    }

    public void run(double dt) {
    }

    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    private void setSensors() {
        DIODataJNI.setValue(whisker_left_a1_io_, whisker_left_a1_value_) ;
        DIODataJNI.setValue(whisker_left_a2_io_, whisker_left_a2_value_) ;
        DIODataJNI.setValue(whisker_right_a1_io_, whisker_right_a1_value_) ;
        DIODataJNI.setValue(whisker_right_a2_io_, whisker_right_a2_value_) ;
        DIODataJNI.setValue(whisker_left_b_io_, whisker_left_b_value_) ;
        DIODataJNI.setValue(whisker_right_b_io_, whisker_right_b_value_) ;
    }
}
