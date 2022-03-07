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
    private SimpleWidget hoodwidget_ ;
    private SetShooterAction shoot_action_ ;
    private double w1current_ ;
    private double hoodcurrent_ ;

    public GPMShooterTestAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        shoot_action_ = new SetShooterAction(sub.getShooter(), 0.0, 0.0, 0.0) ;

        w1widget_ = makeWidget("velocity") ;
        hoodwidget_ = makeWidget("hood") ;

        w1current_ = 0.0 ;
        hoodcurrent_ = 12.0 ;
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
        double hood = hoodwidget_.getEntry().getDouble(hoodcurrent_) ;

        if (hood < 1)
            hood = 1 ;
        else if (hood > 21.0)
            hood = 21.0 ;

        shoot_action_.update(w1, w1, hood) ;
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
