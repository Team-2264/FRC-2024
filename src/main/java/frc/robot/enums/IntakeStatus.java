package frc.robot.enums;

/**
 * Enum for the status of the intake.
 * 
 */
public enum IntakeStatus {
    STOPPED, INTAKING, OUTTAKING;

    /**
     * Returns the speed the intake should be at for the given state.
     * 
     * @return The speed the intake should be at.
     */
    public double speed() {
        return switch(this){
            case STOPPED -> 0;
            case INTAKING -> 0.6;
            case OUTTAKING -> -0.6;
    
        };
    
    }



}
