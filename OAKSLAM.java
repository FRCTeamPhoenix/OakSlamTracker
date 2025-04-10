package frc.robot.subsystems.oakslam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class OAKSLAM extends SubsystemBase {

  private final SLAMConsumer consumer;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("OAKSLAM");
  private DoubleArrayTopic topic = table.getDoubleArrayTopic("Pose");
  private final double[] zero = {0, 0, 0};
  private DoubleArraySubscriber subscriber = topic.subscribe(zero);

  public OAKSLAM(SLAMConsumer consumer) {
    this.consumer = consumer;
  }

  @Override
  public void periodic() {
    TimestampedDoubleArray[] array = subscriber.readQueue();
    for (TimestampedDoubleArray measurement : array) {
      if (measurement.serverTime < 0) {
        continue;
      }
      Pose2d pose =
          new Pose2d(
              measurement.value[0],
              measurement.value[1], Rotation2d.kZero);
      Logger.recordOutput("OAKSLAM/Pose", pose);
      consumer.accept(pose, measurement.serverTime);
    }
  }

  @FunctionalInterface
  public static interface SLAMConsumer {
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds);
  }
}
