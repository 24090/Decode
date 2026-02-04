package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

@Autonomous(name="Red - Close - 15 Sorted", group="Auto")
class Pattern15Red: Pattern15(true)

@Autonomous(name="Blue - Close - 15 Sorted", group="Push")
class Pattern15Blue: Pattern15(false)
open class Pattern15(isRed: Boolean): Auto(
    isRed,
    farStartPose,
    {
        Race(
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
                    intake.spinUp(),
                    Instant { shooter.setTargetVelocityFromDistance(closeDistance) },
                    closeShootCycle(),
                ),
                shooter.stop(),
                grabBallCycle(1),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                closeShootCycle(),

                shooter.stop(),
                fromRampCycle(),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                closeShootCycle(),

                shooter.stop(),
                fromRampCycle(),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                closeShootCycle(),

//                shooter.stop(),
//                grabBallCycle(0, isRed, intake, drive),
//                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
//                closeShootCycle(),

                shooter.stop(),
                grabBallCycle(2),
                Instant{shooter.setTargetVelocityFromDistance(closeDistance)},
                leaveShootCycle(),

                name = "Auto"
            ),
            Forever({
                drive.update(); recordTime("drive")
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }, "Writes")
        )
    }
)