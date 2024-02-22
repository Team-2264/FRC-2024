package frc.robot.enums;

/**
 * Enum for the arm status. Represents what the arm is currently doing.
 * 
 */
public enum ArmState {
    INTAKE, AMP, START, HOME, MANUAL_SHOOT, LOCKED;

    /**
     * Returns the angle the shoulder should be at for the given state.
     * 
     * @return The angle the shoulder should be at in rotations.
     */
    public double shoulderAngle() {
        double degrees = switch(this){
            case INTAKE -> 0; 
            case AMP -> 100;
            case START -> 70;
            case HOME -> 2;
            case MANUAL_SHOOT -> 20;
            case LOCKED -> 0; // No angle while locked
    
        };

        return degrees / 360.0;
    
    }

}
