package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.RepeatCommandUntil
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.util.timeSeconds

@Autonomous(name="PartnerAutoRed", group="Auto")
class PartnerAutoRed: PartnerAuto(true)

@Autonomous(name="PartnerAutoBlue", group="Auto")
class PartnerAutoBlue: PartnerAuto(false)

open class PartnerAuto(val isRed: Boolean): Auto(isRed, farStartPose, {Race(
    Forever({
        recordTime("other")
        reads.update()
        recordTime("reads")
        telemetry.addData("currentPose", drive.localizer.pose)
    }, "Reads" ),
    ForeverCommand {
        Sequence(
            Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Far.distance) },
            farShootCycle(),
            shooter.stop(),
            loadZoneCycle(),
        )
    },
    Forever({
        drive.update(); recordTime("drive")
        shooter.update(); recordTime("shooter")
        intake.update(); recordTime("intake")
        telemetry.update()
    }, "Writes")
)})
