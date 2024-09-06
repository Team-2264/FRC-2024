## Team 2264's robot code for the 2024 "Crescendo" Challenge
The programming team had one major goal this year: to keep our code clean and documented so that future team members and observers would be able to learn and grow from it. This completed README file is our way of completing this goal.

## Robot Overview
Our robot has a swerve-drive drivetrain operated by a PS5 controller. It can auto-target and shoot on the speaker, efficiently score notes into the amplifier, and climb/hang on the chain.

## Project Structure
Here is a basic layout of the file structure of the project
- `src`
    - `lib`:  All code not directly involved in controlling the robot
        - motor wrappers
        - arm trajectory math
        - other random helpers
        - etc

    - `robot`: All main code
        - `commands` / `cmdGroups`: All commands
        - `enums`: All enums
        - `subsystems`: All subsystems
        - constants,
        - swerve-drive
        - etc

## Key Feature: Vision
The robot's vision system uses PhotonVision on a Raspberry Pi 5 coprocessor to track April tags. The data collected from the tags can be used to estimate the robot's position on the field. This benefits the robot in many ways, including automatically targeting the speaker and properly aligning auto paths. When the system automatically targets the speaker, it handles all necessary adjustments. This includes locking the orientation of the swerve drive to point at the speaker, calculating the proper speed for the flywheels, and estimating the release angle for the note. Our angle estimation considers the effect of gravity and the current flywheel speed to provide the most accurate shot possible.

## Other Features:
- PathPlanner for autonomous
- CMD-Based Codebase

## License
This code is licensed under the GNU General Public License v3.0. Please review and comply with the license terms when using or distributing the code.
