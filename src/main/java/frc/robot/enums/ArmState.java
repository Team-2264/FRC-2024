package frc.robot.enums;

/**
 * Enum for the arm status. Represents what the arm is currently doing.
 * 
 */
public enum ArmState {
    INTAKE, AMP, START, HOME, MANUAL_SHOOT, AUTO_SHOOT;

    /**
     * Returns the angle the shoulder should be at for the given state.
     * 
     * @return The angle the shoulder should be at.
     */
    public double shoulderAngle() {
        return switch(this){
            case INTAKE -> 0;
            case AMP -> 120;
            case START -> 60;
            case HOME -> 20;
            case MANUAL_SHOOT -> 10;
            case AUTO_SHOOT -> 0; // No angle for auto shoot
    
        };
    
    }

}
