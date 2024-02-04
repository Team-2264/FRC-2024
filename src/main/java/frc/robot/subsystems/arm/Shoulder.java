package frc.robot.subsystems.arm;

import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Shoulder {
    private Neo[] motors;

    private double targetAngle;
    
    /**
     * Creates a new Shoulder object.
     * 
     * @param motorConfigs The configurations for the motors on the shoulder.
     */
    public Shoulder(NeoConfiguration[] neoConfigs) {
        // create motors
        motors = new Neo[neoConfigs.length];
        for(int i = 0; i < neoConfigs.length; i++) {
            motors[i] = new Neo(neoConfigs[i]);

        }
        
        targetAngle = 0.0;
        
    }

    /**
     * Rotates the shoulder to the target angle.
     * 
     */
    public void rotateTo(int targetAngle) {
        this.targetAngle = targetAngle;

        for (int i = 0; i < motors.length; i++) {
            motors[i].rotateTo(Conversions.degreesToRevs(targetAngle, Constants.Arm.shoulderRatio));

        }

    }

}
