package frc.robot.utills;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Settings;
import edu.wpi.first.math.trajectory.Trajectory;


public class CustomRamseteControllerAbstraction extends RamseteController{
    
    private Pose2d m_poseError;
    private final double m_b;
    private final double m_zeta;

    
    public CustomRamseteControllerAbstraction(double b, double zeta){
        super(b, zeta);
        m_b = b;
        m_zeta = zeta;
    }
    public CustomRamseteControllerAbstraction(){
        super(2.0, 0.7);
        this.m_b = 2.0;
        this.m_zeta = 0.7;
    }
    
    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
          return 1.0 - 1.0 / 6.0 * x * x;
        } else {
          return Math.sin(x) / x;
        }
      }
    

    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {
                this.m_poseError = poseRef.relativeTo(currentPose);

                // Aliases for equation readability
                final double eX = m_poseError.getTranslation().getX();
                final double eY = m_poseError.getTranslation().getY();
                final double eTheta = m_poseError.getRotation().getRadians();
                final double vRef = linearVelocityRefMeters;
                final double omegaRef = angularVelocityRefRadiansPerSecond;

                SmartDashboard.putNumber("Auto Diag/Current X", currentPose.getTranslation().getX());
                SmartDashboard.putNumber("Auto Diag/Reference X", poseRef.getTranslation().getX());

                SmartDashboard.putNumber("Auto Diag/eX", eX);
                SmartDashboard.putNumber("Auto Diag/eY", eY);
                SmartDashboard.putNumber("Auto Diag/eTheta", eTheta);
                SmartDashboard.putNumber("Auto Diag/vRef", vRef);
                SmartDashboard.putNumber("Auto Diag/omegaRef", omegaRef);
                

            
                double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2));

                SmartDashboard.putNumber("Auto Diag/k",k);

                SmartDashboard.putNumber("Auto Diag/vX [m per s]",vRef * m_poseError.getRotation().getCos() + k * eX);
                SmartDashboard.putNumber("Auto Diag/vY [m per s]", 0.0);
                SmartDashboard.putNumber("Auto Diag/vOmega [rad per s]", omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);

                SmartDashboard.putNumber("Auto Diag/vX [t per 100ms]", Conversions.convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(vRef * m_poseError.getRotation().getCos() + k * eX, Settings.Drivetrain.Encoders.WHEEL_DIAMETER, false, Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION));
            
                return new ChassisSpeeds(vRef * m_poseError.getRotation().getCos() + k * eX,
                                         0.0,
                                         omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);
        
    }
    public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState) {
        return calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter);
      }
    
}
