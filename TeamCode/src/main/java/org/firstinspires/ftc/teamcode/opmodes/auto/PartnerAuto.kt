package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose

@Autonomous(name="PartnerAutoRed", group="Auto")
class PartnerAutoRed: PartnerAuto(true)

@Autonomous(name="PartnerAutoBlue", group="Auto")
class PartnerAutoBlue: PartnerAuto(false)

open class PartnerAuto(val isRed: Boolean): Auto(isRed, farStartPose, {Race(
    Forever({
        recordTime("other")
        reads.update();
        recordTime("reads")
        telemetry.addData("currentPose", drive.localizer.pose)
    }, "Reads" ),
    Sequence(
        Instant{
            shooter.targetVelocityLeft = 1820.0
            shooter.targetVelocityRight = 1820.0
       },
        farShootCycle(),
        ForeverCommand({
            Sequence(
                shooter.stop(),
                loadZoneCycle(),
                Instant { shooter.setTargetVelocityFromDistance(farDistance) },
                farShootCycle(),
            )
        })
    ),
    Forever({
        drive.update(); recordTime("drive")
        shooter.update(); recordTime("shooter")
        intake.update(); recordTime("intake")
        telemetry.update()
    }, "Writes")
)})
