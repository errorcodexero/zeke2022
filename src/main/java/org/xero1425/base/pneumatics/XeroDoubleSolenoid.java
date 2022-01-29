package org.xero1425.base.pneumatics;

import org.xero1425.base.XeroRobot;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class XeroDoubleSolenoid {
    private DoubleSolenoid solenoid_ ;
    private SimDevice simdev_ ;
    private SimInt simstate_ ;
    private DoubleSolenoid.Value state_ ;

    final private String SimDeviceName = "Solenoid" ;
    final private String SimeStateName = "state" ;

    public XeroDoubleSolenoid(XeroRobot robot, int module, int forward, int reverse) {
        if (XeroRobot.isSimulation()) {
            simdev_ = SimDevice.create(SimDeviceName, calcIndex(module, forward)) ;
            simstate_ = simdev_.createInt(SimeStateName, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(module, robot.getPneumaticsType(), forward, reverse) ;
        }

        state_ = DoubleSolenoid.Value.kOff ;
    }

    public XeroDoubleSolenoid(XeroRobot robot, int forward, int reverse) {
        if (XeroRobot.isSimulation()) {
            simdev_ = SimDevice.create(SimDeviceName, calcIndex(0, forward)) ;
            simstate_ = simdev_.createInt(SimeStateName, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(robot.getPneumaticsType(), forward, reverse) ;
        }
    }

    public void close() {
        if (!XeroRobot.isSimulation()) {
            solenoid_.close() ;
        }
    }

    public void set(DoubleSolenoid.Value value) {
        state_ = value ;

        if (XeroRobot.isSimulation()) {
            simstate_.set(value.ordinal());
        }
        else {
            solenoid_.set(value) ;
        }
    }

    public DoubleSolenoid.Value get() {
        return state_ ;
    }

    public void toggle() {
        if (state_ == DoubleSolenoid.Value.kForward) {
            set(DoubleSolenoid.Value.kReverse) ;
        }
        else if (state_ == DoubleSolenoid.Value.kReverse) {
            set(DoubleSolenoid.Value.kForward) ;
        }
    }

    public boolean isFwdSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isFwdSolenoidDisabled() ;
        }

        return ret ;
    }

    public boolean isRevSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isRevSolenoidDisabled() ;
        }

        return ret ;
    }

    private int calcIndex(int module, int channel) {
        return module * 32 + channel ;
    }
}
