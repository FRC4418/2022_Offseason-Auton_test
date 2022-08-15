package frc.robot.constants;


// import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Drivetrain {
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.45);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.00);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.00);
        
        SmartNumber MAX_SPEED_ANGLE = new SmartNumber("Driver Settings/Max Speed Angle", 0.5);
        SmartNumber MAX_SPEED = new SmartNumber("Driver Settings/Max Speed", 0.5);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 2.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver Settings/Speed Filtering", 0.125);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.005);

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(26.9); // SEAN PROMISED !

        public interface Motion {

            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

            public interface FeedForward {
                double kS = 0.20094;
                double kV = 1.6658;
                double kA = 0.4515;
            }

            public interface PID {
                double kSlot = 0;
                double kF = 0;
                double kP = 0;
                double kI = 0;
                double kD = 0;
                double kTimeoutMs = 50;
            }
        }

        // Encoder Constants
        public interface Encoders {

            public interface GearRatio {

                public interface Stages {
                    double FIRST_STAGE = (11.0 / 50.0);

                    double SECOND_STAGE = (34.0 / 50.0);
                }

                double ENCODER_TO_WHEEL = Stages.FIRST_STAGE * Stages.SECOND_STAGE;
            }

            double WHEEL_DIAMETER = Units.inchesToMeters(4);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double ENCODER_PULSES_PER_REVOLUTION = 4096;
            double ENCODER_DISTANCE_PER_PULSE =
                    (WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REVOLUTION)
                            * GearRatio.ENCODER_TO_WHEEL;
        }
    }

    public interface Feeder {
        // Low Pass Filter and deadband for Feeder Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Feeder Settings/Speed Deadband", 0.00);

        SmartNumber SPEED_POWER = new SmartNumber("Feeder Settings/Speed Power", 1.0);

        SmartNumber MAX_SPEED = new SmartNumber("Feeder Settings/Max Speed", 0.7);

        SmartNumber MIN_SPEED = new SmartNumber("Feeder Settings/Min Speed", 0);

        SmartNumber SPEED_FILTER = new SmartNumber("Feeder Settings/Speed Filtering", 0.03);
    }

    public interface Shooter {
        // Low Pass Filter and deadband for Feeder Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Shooter Settings/Speed Deadband", 0.00);

        SmartNumber SPEED_POWER = new SmartNumber("Shooter Settings/Speed Power", 1.0);

        SmartNumber MAX_SPEED = new SmartNumber("Shooter Settings/Max Speed", 0.5);

        SmartNumber MIN_SPEED = new SmartNumber("Shooter Settings/Min Speed", 0);

        SmartNumber SPEED_FILTER = new SmartNumber("Shooter Settings/Speed Filtering", 0.125);
    }

}
