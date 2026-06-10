package org.firstinspires.ftc.teamcode.opmodes.auto


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.getPurePursuit
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI
import kotlin.math.max

@Autonomous(name="Red - Close - 18 indiscriminate", group="Auto")
class Full18Red: Full18(true)

@Autonomous(name="Blue - Close - 18 indiscriminate", group="Auto")
class Full18Blue: Full18(false)
open class Full18(isRed: Boolean): Auto(
    isRed,
    closeStartPose,
    {Race(
        Forever({
            recordTime("other")
            reads.update()
            storedPose = drive.localizer.pose
            storedPattern = indexTracker.pattern
            recordTime("reads")
            telemetry.addData("currentPose", drive.localizer.pose)
        }, "Reads" ),
        Sequence(
           Parallel(
                Instant {
                    //camera.initPattern()
                    drive.follow = getPurePursuit(
                        PurePursuitPath(
                            listOf(closeStartPose.mirroredIf(red), ShootPose.Close.mirroredIf(red)),
                            listOf(HeadingBehaviour.Snap),
                            listOf(5.0),
                        ),
                        drive.localizer
                    )
                },
               Race(
                    Forever {
                        shooter.setHoodAngleAndVelocityFromDistance(max(
                            getScoreDistance((drive.localizer.pose.vector() + drive.localizer.poseVel.vector() * 0.4), red),
                            48.0
                        ))
                    },
                    shootAll()
                )
            ),

            shooter.stop(),
            spikeIntakeCycleClose(1),
            closeShootCycle(),

            shooter.stop(),
            gateIntakeCycleClose(),
            closeShootCycle(),

            shooter.stop(),
            spikeIntakeCycleClose(2),
            closeShootCycle(),

            shooter.stop(),
            gateIntakeCycleClose(),
            closeShootCycle(),

            gateIntakeCycleClose(),
            closeShootCycle(),
            drive.goToCircle(Pose(70.0, 40.0, PI/2).mirroredIf(red)),

            name = "Auto"
        ),
        Forever({
            drive.update(); recordTime("drive")
            shooter.update(); recordTime("shooter")
            intake.update(); recordTime("intake")
            telemetry.update()
        }, "Writes")
    )}
)