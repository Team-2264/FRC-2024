package frc.robot;

import frc.lib.motors.Neo;
import frc.robot.enums.ShoulderPosition;

public class Shoulder {
    public ShoulderPosition shoulderStatus;
    Neo[] motorsArray = new Neo[Constants.Arm.motorConfigs.length];
    
    public Shoulder(){
        for(int i = 0; i < motorsArray.length; i++) {
            motorsArray[i] = new Neo(Constants.Arm.motorConfigs[i]);
        }
        shoulderStatus = ShoulderPosition.Bottom;
    }

    // rotate motors to reach a specific position based on the angle parameter
    public void massRotateMotors (int targetAngle) {
        for (int i = 0; i < motorsArray.length; i++) {
            motorsArray[i].rotateTo(((double) targetAngle) / 360.0 * Constants.Arm.shoulderRatio);
        }
    }

    // angle TBD
    public void goToPosition(ShoulderPosition targetPos) {
        if (shoulderStatus!=targetPos){
            massRotateMotors(targetPos.getAngle());
        }
    }
}
