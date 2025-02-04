Hi all, I will not be here this weekend, so here is a guide for everything
that we need to get done, prioritizing the stuff that I dont need to be there for

TODO:
1. configure and tune swerve


a. configure swerve

Follow this guide for configuring swerve and tuning basic parameters
https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template



Drivetrain:
ROBOT_MASS_KG
https://pathplanner.dev/robot-config.html#module-config-options
ROBOT_MOI
WHEEL_COF




2. constants

algae intake:
ALGAE_INTAKE_HEIGHT: distance from

coral intake:
CORAL_INTAKE_HEIGHT



Constants to find (NOT gains):




arm:
TOP_UPPER_LIMIT
TOP_LOWER_LIMIT
BOTTOM_UPPER_LIMIT
BOTTOM_LOWER_LIMIT
ARM_MOTOR_INVERTED
LENGTH_TO_END_EFFECTOR

Elevator:
MIN_HEIGHT_INTAKE_AVOIDANCE
MIN_HEIGHT_BOTTOM_AVOIDANCE
ELEVATOR_MAX_HEIGHT
ELEVATOR_MAX_ROTATIONS


ARM TUNING:
Verify inputs in disable,

tune kg
arm should be able to just barely hold itself up while horizontal
verify at other points


    /*
     * TUNING GUIDE:
     * Go in this order
     * set all gains to 0
     *
     * kG:
     * kG is the term that accounts for gravity, note that gravity impacts the arm
     * more
     * when it is further out, because the arm produces more torque.
     * To tune:
     * Increase kG until arm can *just barely* hold itself up without falling down
     *
     * kS:
     * kS is the term that accounts for static friction in the gearbox,
     * this term effectively cancels out friction, so that we dont have to
     * worry about it later on
     *
     * to tune:
     * increase kS to as high as possible without the arm moving
     *
     * kV:
     * since we are using torque-current control, we control acceleration directly,
     * and dont
     * need a kV term
     *
     * kA:
     * acceleration term, jonah says we dont need it so im inclined to beleive him
     * for now
     *
     */

     Activate one subsystem at a time by switching out IO implementations.
Update constants and check all sensor values while the robot is disabled, along with peripheral functionality like the coast override switch.
Verify basic movement in open-loop to check for any obvious mechanical or electrical issues.
For closed-loop mechanisms, run any feed forward characterization routines (we use custom commands for this rather than SysId, so it’s typically a very fast process).
Reset PID gains and trajectory constraints to very small values, then tune the mechanism controller by slowing making them more aggressive.
Tune high-level controls like setpoints and sequencing logic.


For mechanisms with a chance of breaking themselves, we also rely on additional measures like the automatic E-stop on our 2023 arm. That feature was critical to bringing up that mechanism without causing damage.
