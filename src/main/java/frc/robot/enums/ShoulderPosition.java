package frc.robot.enums;

public enum ShoulderPosition {
        Top, // start position
        Middle, 
        Bottom; // intake position

        public int getAngle() {
                return switch(this){
                        case Top -> 60;
                        case Middle -> 45;
                        case Bottom -> 10;
                };
        }

}
