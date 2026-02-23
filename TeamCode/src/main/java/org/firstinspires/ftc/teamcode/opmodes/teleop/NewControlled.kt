package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.Teleop
import org.firstinspires.ftc.teamcode.opmodes.poses.farStartPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreAngle
import org.firstinspires.ftc.teamcode.opmodes.poses.getScoreDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.getScorePose
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.subsystems.drive.getHeadingLockTeleop
import org.firstinspires.ftc.teamcode.subsystems.drive.getStopPosition
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopFollower
import org.firstinspires.ftc.teamcode.subsystems.drive.getTeleopTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.util.Reference
import org.firstinspires.ftc.teamcode.util.predictedShootPosition
import org.firstinspires.ftc.teamcode.util.storedPattern
import org.firstinspires.ftc.teamcode.util.storedPose
import kotlin.math.PI

@TeleOp(group = "A", name="NewControlled")
class NewControlled: Teleop( { opmode ->
    val dashboard = FtcDashboard.getInstance()
    val isRed = Reference(false)
    var useStoredPose = false
    val lastLockHeading = Reference(false)
    val lastLockTranslational = Reference(false)
    val targetPose = Reference(Pose(0.0, 0.0, 0.0))
    var shootingMode = false
    var patternMode = false

    drive.localizer.pinpoint.recalibrateIMU()

    var time = System.currentTimeMillis()
    val recordTime = { name:String ->
        val newTime = System.currentTimeMillis()
        telemetry.addData("$name (ms)",  newTime - time)
        time = newTime
    }

    while (opmode.opModeInInit()){
        if (opmode.gamepad2.guideWasPressed()) {
            isRed.set(!isRed.get())
        }
        if (opmode.gamepad2.backWasPressed()) {
            useStoredPose = !useStoredPose
        }
        telemetry.addData("isRed? (center button/guide)", isRed)
        telemetry.addData("useStoredPose? (back)", useStoredPose)
        telemetry.update()
    }

    indexTracker.pattern = storedPattern ?: indexTracker.pattern
    camera.initLocalize()

    val getPredictedPosition = {predictedShootPosition(
        intake.getMinShootTime(),
        drive.localizer.pose,
        drive.localizer.poseVel,
        drive.estimateAcceleration()
    )}
    val setShooters = { position: Vector ->
        shooter.setTargetVelocityFromDistance(getScoreDistance(position, isRed.get()))
    }

    val translationalFunction = getTeleopTranslational(opmode.gamepad1, drive.localizer, lastLockTranslational, targetPose, isRed)
    val normalFollow = getTeleopFollower(opmode.gamepad1, drive.localizer, isRed, lastLockHeading, targetPose, translationalFunction)
    val headingLockFollow = getHeadingLockTeleop({ getScoreAngle(getPredictedPosition(), isRed.get()) }, opmode.gamepad1, drive.localizer, translationalFunction, isRed, targetPose)


    val changeShootingMode = {
        shootingMode = !shootingMode
        drive.follow = if (shootingMode) headingLockFollow else normalFollow
        opmode.gamepad1.rumble(1.0, 1.0, 150)
    }

    drive.follow = normalFollow

    val relocalize = {
        val cameraPose = camera.getPose()
        if (cameraPose != null){
           // drive.localizer.pose = Pose(drive.localizer.x * 6.0/7.0 + cameraPose.x * 1.0/7.0, drive.localizer.y * 6.0/7.0 + cameraPose.y * 1.0/7.0, drive.localizer.heading)
        }
    }

    val updateP2 = {
        if (opmode.gamepad2.aWasPressed()){
            patternMode = !patternMode
        }
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
        telemetry.addData("red? (center button/guide)", isRed.get())
        telemetry.addData("rampCount (x and y)", indexTracker.rampCount)
        telemetry.addData("patternMode (a)", patternMode)
        telemetry.addData("pattern", indexTracker.pattern)
        telemetry.addLine("--------------------------")
    }

    val parkButton = { pose: Pose, wasPressed: () -> Boolean, isPressed: () -> Boolean ->
        if (wasPressed()){
            shooter.setTargetVelocities(0.0)
            targetPose.set(pose.mirroredIf(isRed.get()))
            runBlocking(Race(
                WaitUntil { !isPressed() },
                Forever {
                    reads.update()
                    updateP2()
                },
                Sequence(
                    drive.goToCircle(pose.mirroredIf(isRed.get())),
                ),
                Forever {
                    intake.update()
                    drive.update()
                    shooter.update()
                    telemetry.update()
                }
            ))
            drive.follow = normalFollow
        }
    }

//    drive.localizer.pose =
//        if (useStoredPose)
//            storedPose ?: farStartPose.mirroredIf(isRed.get())
//        else
//            farStartPose.mirroredIf(isRed.get())

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
        if (opmode.gamepad1.rightBumperWasPressed()){
            intake.behaviour = Intake.IntakeBehaviour.Eject
        }
        if (opmode.gamepad1.rightBumperWasReleased()){
            intake.behaviour = Intake.IntakeBehaviour.Grab
        }

        parkButton(
            Pose(36.0, -42.0 + robotLength/2.0, -PI/2),
            opmode.gamepad1::dpadDownWasPressed, {opmode.gamepad1.dpad_down}
        )
        parkButton(
            Pose(36.0, -24.0 - robotLength/2.0, PI/2),
            opmode.gamepad1::dpadUpWasPressed, {opmode.gamepad1.dpad_up}
        )
        parkButton(
            Pose(42.0 - robotLength/2.0, -36.0, 0.0),
            if (!isRed.get()) opmode.gamepad1::dpadLeftWasPressed else opmode.gamepad1::dpadRightWasPressed, {if (!isRed.get()) opmode.gamepad1.dpad_left else opmode.gamepad1.dpad_right}
        )
        parkButton(
            Pose(24.0 + robotLength/2.0, -36.0, PI),
            if (!isRed.get()) opmode.gamepad1::dpadRightWasPressed else opmode.gamepad1::dpadLeftWasPressed, {if (!isRed.get()) opmode.gamepad1.dpad_right else opmode.gamepad1.dpad_left}
        )

        if (inLaunchZone(drive.localizer.pose, threshold = 1.5) && shootingMode && getScoreDistance(drive.localizer.pose.vector(), isRed.get()) > 50.0) {
            runBlocking(Race(
                WaitUntil { opmode.gamepad1.leftBumperWasPressed() || !inLaunchZone(drive.localizer.pose, -1.5) },
                Forever {
                    val packet = TelemetryPacket()
                    reads.update()
                    updateP2()
                },
                Sequence(
                    Future {
                        targetPose.set(
                            getScorePose(
                                getStopPosition(
                                    drive.localizer.pose,
                                    drive.localizer.poseVel
                                ),
                                isRed.get()
                            )
                        )
                        shooter.setTargetVelocityFromDistance(getScoreDistance(targetPose.get().vector(), isRed.get()))
                        return@Future drive.goToCircle(targetPose.get())
                    },
                    Parallel(
                        Instant {relocalize()} ,
                        Future {
                            if (patternMode){
                                shootPattern()
                            } else {
                                shootAll()
                            }
                        }
                    )
                ),
                Forever {
                    intake.update()
                    drive.update()
                    shooter.update()
                    telemetry.addData("Pose 2:", "${drive.localizer.pose} (${targetPose.get()})")
                    telemetry.addData("Shooter Velocity: ", "L: ${shooter.motorLeft.velocity}(${shooter.targetVelocityLeft}) R: ${shooter.motorRight.velocity}(${shooter.targetVelocityRight})")
                    telemetry.update()
                }
            )
            )
            changeShootingMode()
            intake.spinUp().update()
        }

        if (opmode.gamepad1.leftBumperWasPressed()){
            changeShootingMode()
        }

        drive.update()
        if (shootingMode) (
            setShooters(getPredictedPosition())
        ) else {
            shooter.setTargetVelocities(0.0)
        }
        shooter.update()
        intake.update()
        telemetry.addData("pose", drive.localizer.pose)
        telemetry.addData("Shooter Velocity: ", "L: ${shooter.motorLeft.velocity}(${shooter.targetVelocityLeft}) R: ${shooter.motorRight.velocity}(${shooter.targetVelocityRight})")
        telemetry.update()
    }
})