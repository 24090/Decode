package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.PI

@Autonomous(name = "Ri24h Auto")
class AutoRi24h: LinearOpMode() {
    override fun runOpMode() {
        val bulkReads = BulkReads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val farShootCycle = {Sequence(
            drive.goTo(Pose(12.0, 12.0, 0.4)),
            intake.releaseBall(),
            intake.releaseBall(),
            intake.releaseBall(),
            name = "FarShootCycle"
        )}

        val grabBallCycle = {n: Int -> Sequence(
            intake.spinUp(),
            drive.goTo(Pose(36.0 + 24 * n, 24.0, PI/2)),
            drive.goTo(Pose(36.0 + 24 * n, 60.0, PI/2)),
            intake.spinDown(),
            name = "GrabBallCycle $n"
        )}
        waitForStart()
        drive.localizer.pose = Pose(12.0, 12.0, PI/4)
        runBlocking(Race(
            Forever {
                bulkReads.update()
                drive.update()
                shooter.update()
                intake.update()
            },
            Sequence(
                farShootCycle(),
                grabBallCycle(0),
                farShootCycle(),
                grabBallCycle(1),
                farShootCycle(),
                grabBallCycle(2),
                farShootCycle()
            )
        ))
    }
}
