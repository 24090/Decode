package org.firstinspires.ftc.teamcode.opmodes.auto

import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

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
                intake.spinUp(),
                Instant {
                    shooter.setTargetVelocityFromDistance(closeDistance)
                    camera.initPattern()
                },
                closeShootCycle(),
            ),

            shooter.stop(),
            grabAndOpenCycleClose(),
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
    )}
)