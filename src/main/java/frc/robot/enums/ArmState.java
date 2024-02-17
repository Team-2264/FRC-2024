package frc.robot.enums;

/**
 * Enum for the arm status. Represents what the arm is currently doing.
 * 
 */
public enum ArmState {
    INTAKE, AMP, START, HOME, CLIMB, CUSTOM, MANUAL_SHOOT;

    /**
     * Returns the angle the shoulder should be at for the given state.
     * 
     * @return The angle the shoulder should be at in rotations.
     */
    public double shoulderAngle() {
        double degrees = switch(this){
            case INTAKE -> 0;
            case AMP -> 100;
            case START -> 50;
            case HOME -> 2;
            case CLIMB -> 90;
            case MANUAL_SHOOT -> 15;
            case CUSTOM -> 0; // No angle for custom
    
        };

        return degrees / 360.0;
    
    }

}
