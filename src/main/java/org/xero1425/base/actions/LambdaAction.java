package org.xero1425.base.actions;

import org.xero1425.misc.MessageLogger;

public class LambdaAction extends Action {
    @FunctionalInterface
    public interface ActionFunction 
    {
        void evaluate() ;
    }

    private ActionFunction func_ ;
    private String name_ ;

    public LambdaAction(MessageLogger logger, String name, ActionFunction dowork) {
        super(logger) ;

        name_ = name ;
        func_ = dowork ;
    }

    @Override
    public void start() {
        func_.evaluate() ;
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + name_ ;
    }
}
