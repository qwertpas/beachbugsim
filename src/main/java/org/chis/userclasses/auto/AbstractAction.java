package org.chis.userclasses.auto;

import org.chis.userclasses.SwerveController;

public abstract class AbstractAction {
    public boolean done = false;
    public abstract void runAction(SwerveController swerve);
}