package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.opmodes.commands.shootCycle
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI
import kotlin.math.pow

@TeleOp(name="Controlled")
class Controlled: LinearOpMode() {
    override fun runOpMode() {
        var isRed = false
        var useStoredPose = false
        val reads = Reads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val camera = Camera(hardwareMap)
        val huskyLens = HuskyLens(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.pattern = storedPattern ?: indexTracker.pattern
        camera.initLocalize()
        var lastLockHeading = false
        var lastLockTranslational = false
        val isLockHeading = {
            gamepad1.right_stick_x.toDouble() == 0.0 //&& (drive.localizer.headingVel < 0.02 || lastLockHeading)
        }
        val isLockTranslational = {
            gamepad1.left_stick_x.toDouble() == 0.0 && gamepad1.left_stick_y.toDouble() == 0.0// && ((drive.localizer.xVel < 0.2 && drive.localizer.yVel < 0.2) || lastLockTranslational)
        }
        val updateHeadingOverride = {
            val lockHeading = isLockHeading()
            if (lockHeading){
                if (!lastLockHeading){
                    drive.targetPose.heading = drive.localizer.heading
                }
                drive.updateHeading()
            } else {
                drive.turn = -gamepad1.right_stick_x.toDouble()
            }
            lastLockHeading = lockHeading
        }
        val updateTranslationalOverride = {
            val lockTranslational = isLockTranslational()
            if (lockTranslational) {
                if (!lastLockTranslational) {
                    val maxAccel = if (drive.dError.x < 0) tipAccelForward else tipAccelBackward
                    val t = (-drive.dError.x)/maxAccel
                    val targetPosition = drive.localizer.pose.vector() + Vector.fromPolar(drive.localizer.heading, maxAccel * t.pow(2)/2 + (-drive.dError.x) * t)
                    drive.targetPose.x = targetPosition.x
                    drive.targetPose.y = targetPosition.y
                }
                drive.updateTranslational()
            } else {
                var v = Vector.fromCartesian(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
                v = v * v.length * 1.67
                if (isRed) v = v.rotated(PI)
                drive.strafe = v.rotated(-drive.localizer.heading).y
                drive.drive =  v.rotated(-drive.localizer.heading).x
            }
            lastLockTranslational = isLockTranslational()
        }
        drive.currentUpdateHeading = updateHeadingOverride
        drive.currentUpdateTranslational = updateTranslationalOverride

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
            if (gamepad1.backWasPressed()) {
                drive.localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed)
                drive.targetPose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed)
            }
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
                drive.currentUpdateHeading = drive::updateHeading
                drive.currentUpdateTranslational = drive::updateTranslational
                runBlocking(
                    Race(
                    WaitUntil { !gamepad1.x },
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose =
                            (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                    },
                    Sequence(
                        Parallel(
                            WaitUntil { drive.error.heading < 0.04 && drive.dError.heading < 0.08 },
                        ),
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
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
            }
            if (gamepad1.aWasPressed()) {
                drive.currentUpdateHeading = drive::updateHeading
                drive.currentUpdateTranslational = drive::updateTranslational
                runBlocking(Race(
                    WaitUntil { !gamepad1.a },
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                    },
                    Sequence(
                        Parallel(
                            WaitUntil { drive.error.heading < 0.04 && drive.dError.heading < 0.08 },
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
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
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
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
            }
            if (gamepad1.rightBumperWasPressed()){
                drive.currentUpdateHeading = drive::updateHeading
                drive.currentUpdateTranslational = drive::updateTranslational
                runBlocking(Race(
                    WaitUntil { !gamepad1.right_bumper},
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        drive.targetPose = parkPose
                    },
                    WaitUntil{drive.error.inSquare(1.0,1.0,0.04)},
                    Forever {
                        intake.update()
                        drive.update()
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                        telemetry.update()
                    }
                ))
                drive.currentUpdateHeading = updateHeadingOverride
                drive.currentUpdateTranslational = updateTranslationalOverride
            }

            drive.update()
            shooter.stop().update()
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("dError.x", drive.dError.x)
            telemetry.addData("distance", (scorePosition.mirroredIf(isRed) - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
            telemetry.addData("gamepad1turn", gamepad1.right_stick_x)
            telemetry.addData("gamepad1strafe", gamepad1.left_stick_x)
            telemetry.addData("gamepad1drive", gamepad1.left_stick_y)
            telemetry.update()
        }
    }

}