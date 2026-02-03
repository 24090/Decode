package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI

@Autonomous(name="Red - Push - 12 Sorted", group="Push")
class AutoPushPattern12Red: Pattern15(true)

@Autonomous(name="Blue - Push - 12 Sorted", group="Push")
class AutoPushPattern12Blue: Pattern15(false)
open class AutoPushPattern12(val isRed: Boolean): Auto(
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
                    drive.follow = {
                        DriveVectors.fromTranslation(Vector.fromPolar(-drive.localizer.heading + PI/2, 10.0))
                    }
                    WaitUntil {
                        drive.localizer.pose.mirroredIf(red).y > 24.0
                    }
                }
            ),
            Parallel(
                intake.spinUp(),
                Instant { shooter.setTargetVelocityFromDistance(farDistance) },
                farShootCycle(),
            ),
            shooter.stop(),
            grabBallCycle(1),
            Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
            closeShootCyclePattern(),

            shooter.stop(),
            grabBallCycle(2),
            Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
            closeShootCyclePattern(),

            shooter.stop(),
            grabBallCycle(0),
            Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
            leaveShootCycle(),

//                shooter.stop(),
//                grabBallCycle(0, isRed, intake, drive),
//                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
//                closeShootCycle(),

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