package frc.robot.subsystems;

import frc.lib.motors.Neo;
import frc.robot.Constants;
import frc.robot.Shoulder;
import frc.robot.enums.ShoulderPosition;

public class Arm {
    private Shoulder shoulder;

    public Arm(){
        shoulder = new Shoulder();
    }

    // set shoulde position to bottm + get intake
    public void SetIntake(){
        shoulder.goToPosition(ShoulderPosition.Bottom);
        // call method for getting intakes
    }

    public void ShoulderToMiddle(){
        shoulder.goToPosition(ShoulderPosition.Middle);
    }

    public void ShoulderToTop(){
        shoulder.goToPosition(ShoulderPosition.Top);

    }

}
