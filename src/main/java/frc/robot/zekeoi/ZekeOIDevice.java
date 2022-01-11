package frc.robot.zekeoi;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.OIPanel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ZekeOIDevice extends OIPanel {

    public ZekeOIDevice(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);

        initializeGadgets();
    }

    public void createStaticActions() throws Exception {
    }

    @Override
    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
    }

    private void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
    }
}
