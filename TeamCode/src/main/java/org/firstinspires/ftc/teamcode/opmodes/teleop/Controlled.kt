package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.Teleop
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreAngle
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopFollower
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Reference
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import org.firstinspires.ftc.teamcode.util.storedRed

@TeleOp(name="Controlled")
class Controlled: Teleop({ opmode ->
    val isRed = storedRed
    val targetPose = Reference(Pose(0.0, 0.0, 0.0))
    val lastLockTranslational = Reference(false)
    val lastLockHeading = Reference(false)
    val indexTracker = IndexTracker()

    indexTracker.pattern = storedPattern ?: indexTracker.pattern
    camera.initLocalize()

    val translationalFunction = getTeleopTranslational(opmode.gamepad1, drive.localizer, lastLockTranslational, targetPose, isRed)
    val followOverride = getTeleopFollower(opmode.gamepad1, drive.localizer, isRed, lastLockHeading, targetPose, translationalFunction)
    drive.follow = followOverride

    while (opmode.opModeInInit()){
        if (opmode.gamepad2.guideWasPressed()) {
            isRed.set(!isRed.get())
        }
        telemetry.addData("isRed? (center button/guide)", isRed.get())
        telemetry.update()
    }

    var time = System.currentTimeMillis()
    val recordTime = { name:String ->
        val newTime = System.currentTimeMillis()
        telemetry.addData("$name (ms)", newTime - time)
        time = newTime
    }

    val updateLocalizer = {
            val cameraPose = camera.getPose()
            if (cameraPose != null){
                val diff = Vector.fromPose(cameraPose - drive.localizer.pose).clampedLength(0.5)
                drive.localizer.pose += Pose(diff.x, diff.y, 0.0)
            }

    }

    val updateP2 = {
        if (opmode.gamepad2.guideWasPressed()) {
            isRed.set(!isRed.get())
        }
        if (opmode.gamepad2.xWasPressed()){
            indexTracker.rampCount = ((indexTracker.rampCount + 1)%9 + 9)%9
        }
        if (opmode.gamepad2.yWasPressed()){
            indexTracker.rampCount = ((indexTracker.rampCount - 1)%9 + 9)%9
        }
        telemetry.addLine("--------- FOR P2 ---------")
        telemetry.addData("isRed? (center button/guide)", isRed)
        telemetry.addData("rampCount (x and y)", indexTracker.rampCount)
        telemetry.addData("pattern", indexTracker.pattern)
        telemetry.addLine("--------------------------")
    }
    drive.localizer.pose = storedPose ?: farStartPose
    while (opmode.opModeIsActive()){
        reads.update()
        updateP2()
        recordTime("loop")
        if (opmode.gamepad1.aWasPressed()) {
            runBlocking(intake.spinUp())
        }
        if (opmode.gamepad1.bWasPressed()) {
            runBlocking(intake.stop())
        }
        if (opmode.gamepad2.bWasPressed()){
            runBlocking(intake.stopFront())
        }
        if (opmode.gamepad1.xWasPressed()) {
            runBlocking(
                Race(
                    WaitUntil { !opmode.gamepad1.x },
                    Forever {
                        reads.update()
                        updateP2()
                        val relativePose = (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose))
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        targetPose.get().heading = getScoreAngle(drive.localizer.pose.vector(), isRed.get())
                    },
                    Sequence(
                        drive.goToCircle(Pose(
                            drive.localizer.x, drive.localizer.y,
                            getScoreAngle(drive.localizer.pose.vector(), isRed.get())
                        )),
                        Instant { updateLocalizer() },
                        shootAll(),
                        Parallel(Sleep(0.5), Instant{intake.behaviour = Intake.IntakeBehaviour.Grab}),
                        intake.releaseDual(),
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
        if (opmode.gamepad1.aWasPressed()) {
            val relativePose =
                (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose))
            runBlocking(Race(
                WaitUntil { !opmode.gamepad1.a },
                Forever {
                    reads.update()
                    updateP2()
                    val relativePose = (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose))
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
                    shootPattern()
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
        if (opmode.gamepad1.yWasPressed()) {
            runBlocking(Race(
                WaitUntil { !opmode.gamepad1.y },
                Forever {
                    reads.update()
                    updateP2()
                    val relativePose = (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose))
                    shooter.setTargetVelocityFromDistance(relativePose.length)
                },
                shootAll(),
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
        if (opmode.gamepad1.leftBumperWasPressed()){
            intake.behaviour = Intake.IntakeBehaviour.Eject
        }
        if (opmode.gamepad1.leftBumperWasReleased()){
            intake.behaviour = Intake.IntakeBehaviour.Grab
        }
        if (opmode.gamepad1.rightBumperWasPressed()){
            shooter.targetVelocityLeft = 0.0
            shooter.targetVelocityRight = 0.0
            runBlocking(Race(
                WaitUntil { !opmode.gamepad1.right_bumper},
                Forever {
                    reads.update()
                    updateP2()
                },
                Sequence(
                    drive.goToCircle(parkPose.mirroredIf(!isRed.get()) + Pose(-24.0, 0.0, 0.0)),
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

        drive.update()
        shooter.stop().update()
        shooter.update()
        intake.update()
        telemetry.addData("pose", drive.localizer.pose)
        telemetry.addData("distance", (scorePosition.mirroredIf(isRed.get()) - Vector.fromPose(drive.localizer.pose)).length)
        telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
        telemetry.addData("gamepad1turn", opmode.gamepad1.right_stick_x)
        telemetry.addData("gamepad1strafe", opmode.gamepad1.left_stick_x)
        telemetry.addData("gamepad1drive", opmode.gamepad1.left_stick_y)
        telemetry.update()
    }
}) {
    override fun runOpMode() {

    }

}