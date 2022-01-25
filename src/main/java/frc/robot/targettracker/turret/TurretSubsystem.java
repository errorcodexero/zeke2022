package frc.robot.targettracker.turret;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem ;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class TurretSubsystem extends MotorEncoderSubsystem {
    public static final String SubsystemName = "turret";

    public TurretSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName, false);
    }

    @Override
    public void postHWInit() {
        try {
            setDefaultAction(new MotorEncoderHoldAction(this));
        } catch (MissingParameterException | BadParameterTypeException e) {
        }
    }
}
