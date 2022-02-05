package frc.robot.zekeoi;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.OIPanel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.base.oi.OIPanelButton;

import frc.robot.climber.ClimbAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeOIDevice extends OIPanel {
    boolean has_climber_ ;

    GPMStartCollectAction collect_ ;
    ClimbAction climb_ ;
    FollowTargetAction follow_ ;

    int automode_gadget_ ;
    int collect_v_shoot_gadget_ ;
    int collect_gadget_ ;
    int climb_gadget_ ;
    int climb_lock_gadget_ ;

    public ZekeOIDevice(OISubsystem sub, String name, int index, boolean climber)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);

        has_climber_ = climber ;
        initializeGadgets();
    }

    public void createStaticActions() throws Exception {
        ZekeSubsystem zeke = (ZekeSubsystem)getSubsystem().getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;

        collect_ = new GPMStartCollectAction(gpm) ;
        climb_ = new ClimbAction(zeke.getClimber(), zeke.getTankDrive()) ;
        follow_ = new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker()) ;
    }

    @Override
    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        ZekeSubsystem zeke = (ZekeSubsystem)getSubsystem().getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        ClimberSubsystem climber = zeke.getClimber() ;
        TurretSubsystem turret = zeke.getTurret() ;
        
        if (getValue(collect_v_shoot_gadget_) == 1) {
            if (turret.getAction() != follow_)
                turret.setAction(follow_) ;
        }
        else {
            // Collect mode

            if (getValue(collect_gadget_) == 1) {
                // We want to collect
                if (gpm.getAction() != collect_) {
                    gpm.setAction(collect_) ;
                }
            }
            else {
            }
        }

        if (has_climber_) {
            if (getValue(climb_lock_gadget_) == 0) {
                // The climber is unlocked

                if (getValue(climb_gadget_) == 1) {
                    if (climber.getAction() != climb_)
                        climber.setAction(climb_) ;
                }
            }
        }
    }

    private void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        int num = getSubsystem().getSettingsValue("oi:gadgets:automode").getInteger() ;
        Double [] map = { -0.9, -0.75, -0.5, -0.25, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 } ;
        automode_gadget_ = mapAxisScale(num, map) ;

        num = getSubsystem().getSettingsValue("oi:gadgets:shoot_collect_mode").getInteger() ;
        collect_v_shoot_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("oi:gadgets:collect_onoff").getInteger() ;
        collect_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("oi:gadgets:climb_lock").getInteger() ;
        climb_lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("oi:gadgets:climb").getInteger() ;
        climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.LowToHigh) ;
    }
}
