package org.xero1425.base.actions;

import org.xero1425.misc.MessageLogger;

public class LambdaAction extends Action {
    @FunctionalInterface
    public interface ActionFunction 
    {
        void evaluate() ;
    }

    public interface DoneFunction
    {
        boolean evaluate() ;
    }

    private ActionFunction func_ ;
    private DoneFunction done_ ;
    private String name_ ;

    public LambdaAction(MessageLogger logger, String name, ActionFunction dowork) {
        super(logger) ;

        name_ = name ;
        func_ = dowork ;
        done_ = null ;
    }

    public LambdaAction(MessageLogger logger, String name, ActionFunction dowork, DoneFunction done) {
        super(logger) ;

        name_ = name ;
        func_ = dowork ;
        done_ = done ;
    }

    @Override
    public void start() {
        func_.evaluate() ;
        if (done_ == null || done_.evaluate() == true)
            setDone() ;
    }

    @Override
    public void run() {
        if (done_.evaluate())
            setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + name_ ;
    }
}
