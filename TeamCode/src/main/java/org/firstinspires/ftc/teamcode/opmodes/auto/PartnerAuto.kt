package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.opmodes.commands.loadZoneCycle
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI

@Autonomous(name="PartnerAutoRed", group="Auto")
class PartnerAutoRed: PartnerAuto(true)

@Autonomous(name="PartnerAutoBlue", group="Auto")
class PartnerAutoBlue: PartnerAuto(false)

open class PartnerAuto(val isRed: Boolean): LinearOpMode() {
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val camera = Camera(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.rampCount = 0

        val farShootCycle = {
            Sequence(
                drive.goToCircle(farPose.mirroredIf(isRed), 2.0),
                releasePattern(intake, shooter, huskyLens, indexTracker),
                name = "FarShootCycle"
            )
        }

        while (opModeInInit()){
            indexTracker.pattern = camera.getPattern() ?: indexTracker.pattern
            telemetry.addData("pattern", indexTracker.pattern)
            telemetry.update()
        }
        waitForStart()
        resetRuntime()
        if (!opModeIsActive()){
            return
        }

        drive.localizer.pose = Pose(robotLength/2.0, robotWidth/2.0, 0.0).mirroredIf(isRed)
        drive.targetPose = closePose.mirroredIf(isRed)
        var lastTime = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - lastTime)
            lastTime = newTime
        }

        runBlocking(Race(
            Forever({
                recordTime("other")
                reads.update();
                recordTime("reads")
                telemetry.addData("targetPose", drive.targetPose)
                telemetry.addData("currentPose", drive.localizer.pose)
            }, "Reads" ),
            Sequence(
                Instant{shooter.setTargetVelocityFromDistance(farDistance)},
                farShootCycle(),
                RepeatUntil({
                    Sequence(
                        Instant{shooter.targetVelocity = 0.0},
                        loadZoneCycle(isRed, intake, drive),
                        Instant{shooter.setTargetVelocityFromDistance(farDistance)},
                        farShootCycle(),
                    )
                }, { runtime > 23.0 })
            ),
            Forever({
                drive.update(); recordTime("drive")
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }, "Writes")
        ))
        storedPose = drive.localizer.pose
        storedPattern = indexTracker.pattern
    }
}
