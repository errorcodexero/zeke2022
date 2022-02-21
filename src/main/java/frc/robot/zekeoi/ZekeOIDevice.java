package frc.robot.zekeoi;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OIPanel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.base.oi.OIPanelButton;

import frc.robot.climber.ClimbAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMStopCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeOIDevice extends OIPanel {
    GPMStartCollectAction start_collect_action_;
    GPMStopCollectAction stop_collect_action_;
    GPMFireAction fire_action_;

    GPMStartCollectAction collect_;
    ClimbAction climb_;
    FollowTargetAction follow_;

    int start_collect_gadget_;
    int automode_gadget_;
    int collect_v_shoot_gadget_;
    int climb_gadget_;
    int climb_lock_gadget_;

    int ball1_output_ ;
    int ball2_output_ ;

    int climber_left_a_output_ ;
    int climber_right_a_output_ ;
    int climber_left_b_output_ ;
    int climber_right_b_output_ ;

    public ZekeOIDevice(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);

        initializeGadgets();

        ball1_output_ = sub.getSettingsValue("oi:outputs:ball1").getInteger() ;
        ball2_output_ = sub.getSettingsValue("oi:outputs:ball2").getInteger() ;
        climber_left_a_output_ = sub.getSettingsValue("oi:outputs:climber-left-a").getInteger() ;
        climber_right_a_output_ = sub.getSettingsValue("oi:outputs:climber-right-a").getInteger() ;        
        climber_left_b_output_ = sub.getSettingsValue("oi:outputs:climber-left-b").getInteger() ;
        climber_right_b_output_ = sub.getSettingsValue("oi:outputs:climber-right-b").getInteger() ;                                        
    }

    public void createStaticActions() throws Exception {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();

        start_collect_action_ = new GPMStartCollectAction(gpm);
        stop_collect_action_ = new GPMStopCollectAction(gpm);
        fire_action_ = new GPMFireAction(gpm, zeke.getTargetTracker(), zeke.getTankDrive(), zeke.getTurret()) ;

        collect_ = new GPMStartCollectAction(gpm);
        climb_ = new ClimbAction(zeke.getClimber(), zeke.getTankDrive(), zeke.getOI());
        follow_ = new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker());
    }

    private void setLEDs()
    {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();   
        // ClimberSubsystem climber = zeke.getClimber();

        switch(gpm.getConveyor().getBallCount())
        {
            case 0:
                setOutput(ball1_output_, false);
                setOutput(ball2_output_, false) ;
                break ;
            case 1:
                setOutput(ball1_output_, true);
                setOutput(ball2_output_, false) ;
                break ;    
            case 2:
                setOutput(ball1_output_, true);
                setOutput(ball2_output_, true) ;
                break ;                               
        }        
    }

    @Override
    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        double t = getSubsystem().getRobot().getTime() ;
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();
        ClimberSubsystem climber = zeke.getClimber();
        TurretSubsystem turret = zeke.getTurret();

        setLEDs() ;

        if (getValue(climb_lock_gadget_) == 1) {

            if (turret.getAction() != follow_)
                turret.setAction(follow_);

            if (getValue(collect_v_shoot_gadget_) == 0) {

                // Collect mode
                if (isCollectButtonPressed()) {
                    if (gpm.getAction() != start_collect_action_)
                        gpm.setAction(start_collect_action_);
                } else {
                    if (gpm.getAction() == start_collect_action_)
                        gpm.setAction(stop_collect_action_) ;
                }
            } else {
                if (gpm.getAction() != fire_action_)
                    gpm.setAction(fire_action_);
            }
        } else {
            if (turret.getAction() == follow_)
                turret.setAction(null);

            if (climber != null) {
                if (getValue(climb_gadget_) == 1) {
                    if (climber.getAction() != climb_)
                        climber.setAction(climb_);
                }
            }
        }
    }

    private boolean isCollectButtonPressed() {
        if (getValue(start_collect_gadget_) == 1)
            return true ;

        Gamepad g = getSubsystem().getGamePad() ;
        if (g != null && g.isRTriggerPressed())
            return true ;

        return false ;
    }

    private void initializeGadgets() throws BadParameterTypeException, MissingParameterException {
        int num = getSubsystem().getSettingsValue("oi:gadgets:automode").getInteger();
        Double[] map = { -0.9, -0.75, -0.5, -0.25, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 };
        automode_gadget_ = mapAxisScale(num, map);

        num = getSubsystem().getSettingsValue("oi:gadgets:shoot_collect_mode").getInteger();
        collect_v_shoot_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:collect_onoff").getInteger();
        start_collect_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:climb_lock").getInteger();
        climb_lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:climb").getInteger();
        climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.LowToHigh);
    }
}
