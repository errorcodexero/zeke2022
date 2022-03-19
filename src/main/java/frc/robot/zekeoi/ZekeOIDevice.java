package frc.robot.zekeoi;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OIPanel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.base.oi.OIPanelButton;

import frc.robot.climber.ClimbAction;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.DeployClimberAction;
import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.climber.DeployClimberAction.DeployState;
import frc.robot.gpm.GPMEjectAction;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMManualFireAction;
import frc.robot.gpm.GPMStartCollectAction;
import frc.robot.gpm.GPMStopCollectAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.turret.FollowTargetAction;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class ZekeOIDevice extends OIPanel {
    private GPMStartCollectAction start_collect_action_;
    private GPMStopCollectAction stop_collect_action_;
    private GPMFireAction fire_action_;
    private GPMManualFireAction manual_fire_action_ ;

    private GPMEjectAction eject_action_ ;
    private ClimbAction climb_;
    private DeployClimberAction deploy_climber_ ;
    private DeployClimberAction stow_climber_ ;
    private FollowTargetAction follow_;
    private MotorEncoderGotoAction zero_turret_ ;

    private int start_collect_gadget_;
    private int automode_gadget_;
    private int collect_v_shoot_gadget_;
    private int climb_gadget_;
    private int climb_lock_gadget_;
    private int eject_gadget_ ;
    private int deploy_climb_gadget_ ;
    private int shoot_manual_mode_gadget_ ;

    private int ball1_output_ ;
    private int ball2_output_ ;

    private int limelight_ready_led_ ;
    private int shooter_ready_led_ ;
    private int turret_ready_led_ ;
    private int distance_ok_led_ ;

    private ClimberState climber_state_ ;
    private boolean is_turret_holding_ ;

    private enum ClimberState {
        STARTUP,
        STOWED,
        DEPLOYING,
        DEPLOYED,
        STOWING,
        CLIMBING,
        CLIMBED
    }

    public ZekeOIDevice(OISubsystem sub, String name, int index)
            throws BadParameterTypeException, MissingParameterException {
        super(sub, name, index);

        climber_state_ = ClimberState.STARTUP ;
        is_turret_holding_ = false ;

        initializeGadgets();

        ball1_output_ = sub.getSettingsValue("oi:outputs:ball1").getInteger() ;
        ball2_output_ = sub.getSettingsValue("oi:outputs:ball2").getInteger() ;

        limelight_ready_led_ = sub.getSettingsValue("oi:outputs:shooting:limelight").getInteger() ;
        shooter_ready_led_ = sub.getSettingsValue("oi:outputs:shooting:shooter").getInteger() ;        
        turret_ready_led_ = sub.getSettingsValue("oi:outputs:shooting:turret").getInteger() ;
        distance_ok_led_ = sub.getSettingsValue("oi:outputs:shooting:distance").getInteger() ;                                        
    }
    
    @Override
    public int getAutoModeSelector() {
        int value = getValue(automode_gadget_) ;
        return value ;
    }

    @Override
    public void createStaticActions() throws Exception {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();

        start_collect_action_ = new GPMStartCollectAction(gpm);
        stop_collect_action_ = new GPMStopCollectAction(gpm);
        fire_action_ = new GPMFireAction(gpm, zeke.getTargetTracker(), zeke.getTankDrive(), zeke.getTurret()) ;
        manual_fire_action_ = new GPMManualFireAction(gpm) ;
        eject_action_ = new GPMEjectAction(gpm) ;
        zero_turret_ = new MotorEncoderGotoAction(zeke.getTurret(), 0, true) ;

        if (zeke.getClimber() != null) {
            climb_ = new ClimbAction(zeke.getClimber(), zeke.getTankDrive(), zeke.getOI());
            deploy_climber_ = new DeployClimberAction(zeke.getClimber(), DeployState.Deployed) ;
            stow_climber_ = new DeployClimberAction(zeke.getClimber(), DeployState.Stowed) ;
        }
            
        follow_ = new FollowTargetAction(zeke.getTurret(), zeke.getTargetTracker());
    }

    private void setLEDs()
    {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();   

        switch(gpm.getConveyor().getBallCount())
        {
            case 0:
                setOutput(ball1_output_, true);
                setOutput(ball2_output_, true) ;
                break ;
            case 1:
                setOutput(ball1_output_, false);
                setOutput(ball2_output_, true) ;
                break ;    
            case 2:
                setOutput(ball1_output_, false);
                setOutput(ball2_output_, false) ;
                break ;                               
        }

        if (gpm.getAction() == fire_action_) {
            setOutput(limelight_ready_led_, !fire_action_.hasTarget()) ;
            setOutput(turret_ready_led_, !fire_action_.turretReady()) ;
            setOutput(shooter_ready_led_, !fire_action_.shooterReady()) ;
            setOutput(distance_ok_led_, !fire_action_.distanceOk()) ;

            if (fire_action_.hasTarget() && fire_action_.turretReady() && fire_action_.shooterReady() && fire_action_.distanceOk()) {
                Gamepad g = getSubsystem().getGamePad() ;
                g.rumble(1.0, 2.0) ;
            }
        }
    }

    private void generateCargoActions() {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem();
        TurretSubsystem turret = zeke.getTurret();

        is_turret_holding_ = false ;

        if (turret.getAction() != follow_)
             turret.setAction(follow_);

        if (climber_state_ == ClimberState.STARTUP) {
            zeke.getClimber().setAction(stow_climber_) ;
            climber_state_ = ClimberState.STOWED ;
        }

        if (getValue(collect_v_shoot_gadget_) == 1) {
            //
            // We are collecting
            //
            if (gpm.getAction() == fire_action_) {
                gpm.cancelAction();
            }
            if (isCollectButtonPressed()) {
                if (gpm.getAction() != start_collect_action_)
                    gpm.setAction(start_collect_action_);
            } else {
                if (gpm.getAction() == start_collect_action_)
                    gpm.setAction(stop_collect_action_) ;
            }
        } else {
            //
            // We are shooting
            //
            if (getValue(shoot_manual_mode_gadget_) == 0) {
                //
                // Automatic shooting
                //
                if (gpm.getAction() != fire_action_ && gpm.getConveyor().getBallCount() > 0) {
                    gpm.setAction(fire_action_);
                }
            }
            else {
                if (gpm.getAction() != manual_fire_action_) {
                    gpm.setAction(manual_fire_action_);
                }
            }
        }
    }

    private void generateClimbActions() {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        ClimberSubsystem climber = zeke.getClimber();
        TurretSubsystem turret = zeke.getTurret();
        Gamepad g = getSubsystem().getGamePad() ;

        if (is_turret_holding_ == false) {
            turret.setAction(zero_turret_) ;
            is_turret_holding_ = true ;
        }

        if (climber_state_ == ClimberState.STARTUP) {
            climber.setAction(stow_climber_) ;
            climber_state_ = ClimberState.STOWED ;
        }
        else if (climber_state_ == ClimberState.STOWED) {
            if (getValue(deploy_climb_gadget_) == 1) {
                climber.setAction(deploy_climber_) ;
                climber_state_ = ClimberState.DEPLOYING ;
            }
        }
        else if (climber_state_ == ClimberState.STOWING) {
            if (stow_climber_.isDone()) {                
                climber_state_ = ClimberState.STOWED ;
            } 
            else if (getValue(deploy_climb_gadget_) == 1) {
                climber.setAction(deploy_climber_) ;
                climber_state_ = ClimberState.DEPLOYING ;
            }
        }
        else if (climber_state_ == ClimberState.DEPLOYING) {
            if (deploy_climber_.isDone()) {
                climber.setAction(climb_) ;
                climber_state_ = ClimberState.DEPLOYED ;
            }
            else if (getValue(deploy_climb_gadget_) == 0) {
                zeke.getClimber().setAction(stow_climber_) ;
                climber_state_ = ClimberState.STOWING ;
            }
        }
        else if (climber_state_ == ClimberState.DEPLOYED) {
            if (climb_.pastPointNoReturn()) {
                climber_state_ = ClimberState.CLIMBING ;
            }
            else if (getValue(deploy_climb_gadget_) == 0) {
                climber.setAction(stow_climber_) ;
                climber_state_ = ClimberState.STOWING ;                    
            }
            else if (g.isAPressed()) {
                g.enable();
                climber.setAction(deploy_climber_) ;
                climber_state_ = ClimberState.DEPLOYING ;
            }
            else if (getValue(climb_gadget_) == 1) {
                climb_.setAutomaticDrive() ;
            }
        }
        else if (climber_state_ == ClimberState.CLIMBING)  {
            if (climb_.isDone()) {
                climber_state_ = ClimberState.CLIMBED ;
            }
            else if (getValue(deploy_climb_gadget_) == 0) {
                //
                // If the climb deploy button is switched, we stop the climb as soon as it is
                // safe to do so.
                //
                climb_.stopWhenSafe() ;
            }
        }
        else if (climber_state_ == ClimberState.CLIMBED) {
            //
            // Do nothing, we stay here until the match is over
            //
        }
    }

    @Override
    public void generateActions(SequenceAction seq) throws InvalidActionRequest {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;

        setLEDs() ;

        if (climber_state_ != ClimberState.STOWED) {
            //
            // If we prevously unlocked the climber and got into the climb sequence, we must finish
            // this sequence.  So unless the climber is stowed, we always go process the climber
            //
            generateClimbActions();
        }
        else if (getValue(eject_gadget_) == 1) {
            //
            // The second priority is the eject button.
            //
            if (gpm.getAction() != eject_action_)
                gpm.setAction(eject_action_) ;
        }
        else if (getValue(climb_lock_gadget_) == 1) {
            //
            // The climber is locked, 
            checkGrabbers() ;
            generateCargoActions();
        } else if (zeke.getClimber() != null) {
            generateClimbActions() ;
        }
    }

    private void checkGrabbers() {
        ZekeSubsystem zeke = (ZekeSubsystem) getSubsystem().getRobot().getRobotSubsystem();
        ClimberSubsystem climber = zeke.getClimber() ;

        if (climber.getClampState(WhichClamp.CLAMP_A) != GrabberState.CLOSED) {
            climber.changeClamp(WhichClamp.CLAMP_A, GrabberState.CLOSED);
        }

        if (climber.getClampState(WhichClamp.CLAMP_B) != GrabberState.CLOSED) {
            climber.changeClamp(WhichClamp.CLAMP_B, GrabberState.CLOSED);
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

        num = getSubsystem().getSettingsValue("oi:gadgets:shoot_manual_mode").getInteger() ;
        shoot_manual_mode_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;

        num = getSubsystem().getSettingsValue("oi:gadgets:collect_onoff").getInteger();
        start_collect_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:climb_lock").getInteger();
        climb_lock_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:climb").getInteger();
        climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:eject").getInteger();
        eject_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level);

        num = getSubsystem().getSettingsValue("oi:gadgets:deploy_climber").getInteger() ;
        deploy_climb_gadget_ = mapButton(num, OIPanelButton.ButtonType.Level) ;
    }
}
