package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.opmodes.commands.shootCycle
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopFollower
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI
import kotlin.math.pow

@TeleOp(name="MoveShootControlled")
class MoveShootControlled: LinearOpMode() {
    override fun runOpMode() {
        var isRed = false
        var useStoredPose = false
        var moveShootOutputs: Pair<Double, Double>? = null
        var shootingMode = false

        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val camera = Camera(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.pattern = storedPattern ?: indexTracker.pattern
        camera.initLocalize()
        val followOverride = getTeleopFollower(gamepad1, drive.localizer, { isRed })
        drive.follow = followOverride

        while (opModeInInit()){
            if (gamepad2.guideWasPressed()) {
                isRed = !isRed
            }
            if (gamepad2.backWasPressed()) {
                useStoredPose = !useStoredPose
            }
            telemetry.addData("isRed? (center button/guide)", isRed)
            telemetry.addData("useStoredPose? (back)", useStoredPose)
            telemetry.update()
        }

        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }

        val updateLocalizer = {
//            val cameraPose = camera.getPose()
//            if (cameraPose != null){
//                val diff = Vector.fromPose(cameraPose - drive.localizer.pose).clampedLength(0.5)
//                drive.localizer.pose += Pose(diff.x, diff.y, 0.0)
//            }

        }

        val updateP2 = {
            if (gamepad2.guideWasPressed()) {
                isRed = !isRed
            }
            if (gamepad2.xWasPressed()){
                indexTracker.rampCount = ((indexTracker.rampCount + 1)%9 + 9)%9
            }
            if (gamepad2.yWasPressed()){
                indexTracker.rampCount = ((indexTracker.rampCount - 1)%9 + 9)%9
            }
            telemetry.addLine("--------- FOR P2 ---------")
            telemetry.addData("isRed? (center button/guide)", isRed)
            telemetry.addData("rampCount (x and y)", indexTracker.rampCount)
            telemetry.addData("pattern", indexTracker.pattern)
            telemetry.addLine("--------------------------")
        }
        drive.localizer.pose = if (useStoredPose) storedPose ?: startPose.mirroredIf(isRed) else startPose.mirroredIf(isRed)
        while (opModeIsActive()){
            reads.update()
            updateP2()
            recordTime("loop")
            if (gamepad1.aWasPressed()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasPressed()) {
                runBlocking(intake.stop())
            }
            if (gamepad2.bWasPressed()){
                runBlocking(intake.stopFront())
            }
            if (gamepad1.xWasPressed()) {
                val relativePose = scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose)
                runBlocking(
                    Race(
                        WaitUntil { !gamepad1.x },
                        Forever {
                            reads.update()
                            updateP2()
                            val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                            shooter.setTargetVelocityFromDistance(relativePose.length)
                        },
                        Sequence(
                            drive.goToCircle(Pose(
                                drive.localizer.x, drive.localizer.y,
                                relativePose.angle
                            )),
                            Instant { updateLocalizer() },
                            shootCycle(intake, shooter)
                        ),
                        Forever {
                            intake.update()
                            drive.update()
                            shooter.update()
                            telemetry.addData(
                                "Shooter velocity",
                                (shooter.motorLeft.velocity + shooter.motorRight.velocity) / 2
                            )
                            telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                            telemetry.update()
                        }
                    ))
                runBlocking(intake.spinUp())
                intake.resetPushers()
                drive.follow = followOverride
            }
            if (gamepad1.aWasPressed()) {
                val relativePose =
                    (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                runBlocking(Race(
                    WaitUntil { !gamepad1.a },
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                    },
                    Sequence(
                        Parallel(
                            drive.goToCircle(Pose(
                                drive.localizer.x, drive.localizer.y,
                                relativePose.angle
                            )),
                            shooter.waitForVelocity()
                        ),
                        Instant {updateLocalizer()},
                        releasePattern(intake, shooter, huskyLens , indexTracker)
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                ))
                runBlocking(intake.spinUp())
                intake.resetPushers()
                drive.follow = followOverride
            }
            if (gamepad1.yWasPressed()) {
                runBlocking(Race(
                    WaitUntil { !gamepad1.y },
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                    },
                    shootCycle(intake, shooter),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                ))
                runBlocking(intake.spinUp())
                intake.resetPushers()
                drive.follow = followOverride
            }
            if (gamepad1.rightBumperWasPressed()){
                shooter.targetVelocityLeft = 0.0
                shooter.targetVelocityRight = 0.0
                runBlocking(Race(
                    WaitUntil { !gamepad1.right_bumper},
                    Forever {
                        reads.update()
                        updateP2()
                    },
                    Sequence(
                        drive.goToCircle(parkPose.mirroredIf(!isRed) + Pose(-24.0, 0.0, 0.0)),
                        drive.doWheelie(intake)
                    ),
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                ))
                drive.follow = followOverride
            }
            if (gamepad1.dpadUpWasPressed()){
                shootingMode = !shootingMode
            }

            drive.update()
            shooter.stop().update()
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("distance", (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
            telemetry.addData("gamepad1turn", gamepad1.right_stick_x)
            telemetry.addData("gamepad1strafe", gamepad1.left_stick_x)
            telemetry.addData("gamepad1drive", gamepad1.left_stick_y)
            telemetry.update()
        }
    }

}