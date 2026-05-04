package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.opmodes.commands.Auto
import org.firstinspires.ftc.teamcode.opmodes.commands.Robot
import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose

@Autonomous
class RedRampCycleTesting: RampCycleTesting(true)

@Autonomous
class BlueRampCycleTesting: RampCycleTesting(false)

open class RampCycleTesting(red: Boolean) : Auto(
    red,
    startPose = closeStartPose,
    command = { Race(
        Forever({
            reads.update()
            storedPose = drive.localizer.pose
            storedPattern = indexTracker.pattern
            telemetry.addData("currentPose", drive.localizer.pose)
        }, "Reads" ),
        Sequence(
            Instant {
                intake.behaviour = Intake.IntakeBehaviour.Grab
                shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance)
            },
            closeShootCycle(),
            ForeverCommand { Sequence(
                shooter.stop(),
                gateIntakeCycleClose(),
                Instant { shooter.setHoodAngleAndVelocityFromDistance(ShootPose.Close.distance) },
                closeShootCycle()
            ) }
        ),
        Forever({
            drive.update()
            shooter.update()
            intake.update()
            telemetry.update()
        }, "Writes"))
    }
)