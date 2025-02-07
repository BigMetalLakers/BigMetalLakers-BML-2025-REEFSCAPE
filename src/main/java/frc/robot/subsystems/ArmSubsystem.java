
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    // Create Motors
    public final Spark lifter = new Spark(ArmConstants.kLifterPWMPort);
    public final Spark joint = new Spark(ArmConstants.kJointPWMPort);
    public final Spark coralIntake = new Spark(ArmConstants.kCoralIntakePWMPort);
    public final Spark algaeIntake = new Spark(ArmConstants.kAlgaeIntakePWMPort);

    // Create Encoders
    public final Encoder lifterEncoder = new Encoder(ArmConstants.kLifterEncoderPortA, ArmConstants.kLifterEncoderPortB);
    public final Encoder jointEncoder = new Encoder(ArmConstants.kJointEncoderPortA, ArmConstants.kJointEncoderPortB);

    // Feed Forward & Feed Back Controllers
    public final ElevatorFeedforward lifterFeedForward = new ElevatorFeedforward(0, 0, 0);
    public final PIDController lifterPIDController = new PIDController(1, 0, 0);
    public final ArmFeedforward jointFeedForward = new ArmFeedforward(0, 0, 0);
    public final PIDController jointPIDController = new PIDController(1, 0, 0);

    // The PIDController assumes that the calculate() method is being called 
    // regularly at an interval consistent with the configured period. 
    // Failure to do this will result in unintended loop behavior.


    public void lifterToHeight(double height) {
        double velocity = 0; // change

        lifter.setVoltage(
            lifterFeedForward.calculate(velocity)
            + lifterPIDController.calculate(
                lifterEncoder.getRate(), 
                height)
        );
    }

    public void jointToAngle(double angle) {
        double velocity = 0; // change

        joint.setVoltage(
            jointFeedForward.calculate(angle, velocity)
            + jointPIDController.calculate(
                jointEncoder.getRate(), 
                angle)
        );
    }

    public void lifterBySpeed(double speed) {
        lifter.set(speed);
    }

    public void jointBySpeed(double speed) {
        joint.set(speed);
    }
}