package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.shooter.SetShooterAction;

public class GPMShooterTestAction extends Action {
    private GPMSubsystem sub_ ;
    private SimpleWidget w1widget_ ;
    private SimpleWidget w2widget_ ;
    private SimpleWidget hoodwidget_ ;
    private SetShooterAction shoot_action_ ;
    private double w1current_ ;
    private double w2current_ ;
    private double hoodcurrent_ ;

    public GPMShooterTestAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        shoot_action_ = new SetShooterAction(sub.getShooter(), 0.0, 0.0, 0.0) ;

        w1widget_ = makeWidget("wheel1") ;
        w2widget_ = makeWidget("wheel2") ;
        hoodwidget_ = makeWidget("hood") ;

        w1current_ = 0.0 ;
        w2current_ = 0.0 ;
        hoodcurrent_ = 0.0 ;
    }    

    @Override
    public void start() throws BadMotorRequestException, MotorRequestFailedException {
        sub_.getConveyor().setBypass(true);
        sub_.getConveyor().setMotorsPower(1.0, 1.0);
        sub_.getShooter().setAction(shoot_action_, true) ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        double w1 = w1widget_.getEntry().getDouble(w1current_) ;
        double w2 = w2widget_.getEntry().getDouble(w2current_) ;
        double hood = hoodwidget_.getEntry().getDouble(hoodcurrent_) ;

        shoot_action_.update(w1, w2, hood) ;
    }

    @Override
    public void cancel() {
        sub_.getConveyor().setBypass(false);
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMShooterTestAction" ;
    }

    private SimpleWidget makeWidget(String title) {
        SimpleWidget w = Shuffleboard.getTab("ShooterTuning").add(title, 0.0) ;
        return w.withWidget(BuiltInWidgets.kTextView) ;
    }
}
