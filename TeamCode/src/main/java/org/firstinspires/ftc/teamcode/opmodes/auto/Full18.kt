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
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.minTransfer
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWaitDown
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
            //recordTime("other")
            reads.update()
            storedPose = drive.localizer.pose
            storedPattern = indexTracker.pattern
            //recordTime("reads")
//            telemetry.addData("currentPose", drive.localizer.pose)
        }, "Reads" ),
        Sequence(
            Instant {
                intake.behaviour = Intake.IntakeBehaviour.HoldIdle
                recordTime("start")
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
           Sequence(
               Race(
                   Forever {
                       shooter.setFirstHoodAngleAndVelocityFromDistance(max(
                           getScoreDistance((drive.localizer.pose.vector() + drive.localizer.poseVel.vector() * 0.3), red),
                           48.0
                       ))
                   },
                   Sequence(
                       shooter.waitForRightVelocity(),
                       intake.releaseRight(),
                   )
               ),
               Race(
                   Forever {
                       shooter.setSecondHoodAngleAndVelocityFromDistance(max(
                           getScoreDistance((drive.localizer.pose.vector() + drive.localizer.poseVel.vector() * 0.3), red),
                           48.0
                       ))
                   },
                   Sequence(
                       intake.setAdjustThird(),
                       Sleep(pusherWaitDown),
                       Instant { intake.behaviour = Intake.IntakeBehaviour.TransferQuick },
                       Parallel(
                           shooter.waitForVelocity(),
                           Sleep(minTransfer),
                       ),
                       intake.releaseDual(),
                       Instant {
                           intake.behaviour = Intake.IntakeBehaviour.Grab
                       },
                   )
               )
            ),
            Instant {recordTime("1")},

            shooter.stop(),
            spikeIntakeCycleClose(1),
            shootCycle(ShootPose.Close),
            Instant {recordTime("2")},

            shooter.stop(),
            gateIntakeCycleClose(),
            shootCycle(ShootPose.Close),
            Instant {recordTime("3")},

            shooter.stop(),
            gateIntakeCycle(ShootPose.Close),
            shootCycle(ShootPose.Close),
            Instant {recordTime("5")},

            shooter.stop(),
            gateIntakeCycle(ShootPose.Close),
            shootCycle(ShootPose.Close),
            Instant {recordTime("6")},

            shooter.stop(),
            gateIntakeCycle(ShootPose.Close),
            shootCycle(ShootPose.Close),
            Instant {recordTime("4")},

            shooter.stop(),
            spikeIntakeCycle(2, ShootPose.Closer),
            shootCycle(ShootPose.Closer),
            drive.goToCircle(Pose(70.0, 40.0, PI/2).mirroredIf(red)),
            Instant {
                recordTime("7")
                telemetry.update()
            },
            name = "Auto"
        ),
        Forever({
            drive.update(); //recordTime("drive")
            shooter.update(); //recordTime("shooter")
            intake.update(); //recordTime("intake")
        }, "Writes")
    )}
)