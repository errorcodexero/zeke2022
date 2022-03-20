package frc.robot.climber;

public class ClimberBackupProfile {
    private enum State {
        Startup,
        Delaying,
        Finishing
    }

    private State state_ ;
    private double delay_start_ ;
    private double delay_duration_ ;
    private double threshold_ ;
    private double kp1_ ;
    private double kp2_ ;

    public ClimberBackupProfile(double thresh, double duration, double kp1, double kp2) {
        state_ = State.Startup ;
        delay_duration_ = duration ;
        kp1_ = kp1 ;
        kp2_ = kp2 ;
        threshold_ = thresh ;
        delay_start_ = 0.0 ;
    }

    public double getOutput(double actual, double target, double time) {
        double out = 0.0 ;

        double err = target - actual ;

        switch(state_) {
            case Startup:
                out = err * kp1_ ;
                if (actual < threshold_)
                {
                    delay_start_ = time ;
                    state_ = State.Delaying ;
                }
                break ;
            case Delaying:
                out = 0.0 ;
                if (time - delay_start_ > delay_duration_) {
                    state_ = State.Finishing ;
                }
                break ;
            case Finishing:
                out = err * kp2_ ;
                break ;
        }

        if (out < -0.4)
            out = -0.4 ;
        else if (out > -0.1)
            out = -0.1 ;

        return out ;
    }
}
