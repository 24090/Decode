package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyT
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.processTurnTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getStopPosition
import org.firstinspires.ftc.teamcode.util.Reference
import kotlin.math.PI


fun getTeleopTranslational(
    gamepad: Gamepad,
    localizer: Localizer,
    lastLockTranslational: Reference<Boolean>,
    targetPose: Reference<Pose>,
    isRed: Reference<Boolean>
): (dError: Vector) -> Vector {
    val isLockTranslational = {
        gamepad.left_stick_x.toDouble() == 0.0 && gamepad.left_stick_y.toDouble() == 0.0// && ((drive.localizer.xVel < 0.2 && drive.localizer.yVel < 0.2) || lastLockTranslational)
    }

    return { dError: Vector ->
        val error = getRelativePose(localizer.pose, targetPose.get())
        val lockTranslational = isLockTranslational()
        if (lockTranslational) {
            if (!lastLockTranslational.get()) {
                targetPose.set(getStopPosition(localizer.pose, localizer.poseVel).let{
                    Pose(it.x, it.y, targetPose.get().heading)
                })
            }
            lastLockTranslational.set(true)
            PDLT(error.vector(), dError, xyP, xyD, kS, xyT)
        } else {
            var v = Vector.fromCartesian(-gamepad.left_stick_x.toDouble(), gamepad.left_stick_y.toDouble())
            v = v * v.length * 1.67
            if (!isRed.get()) v = v.rotated(PI)
            lastLockTranslational.set(false)
            v.rotated(-localizer.heading)
        }
    }
}


fun getTeleopFollower(
    gamepad: Gamepad,
    localizer: Localizer,
    isRed: Reference<Boolean>,
    lastLockHeading: Reference<Boolean>,
    targetPose: Reference<Pose>,
    teleopTranslational: (dError: Vector) -> Vector
): () -> DriveVectors {
    val heading = { dError: Pose ->
        if (gamepad.right_stick_x.toDouble() == 0.0){
            if (!lastLockHeading.get()){
                targetPose.set(Pose(targetPose.get().x, targetPose.get().y, localizer.heading))
            }
            lastLockHeading.set(true)
            PDLT(AngleUnit.normalizeRadians(targetPose.get().heading - localizer.heading), dError.heading, hP, hD, kS, hT)
        } else {
            lastLockHeading.set(false)
            -gamepad.right_stick_x.toDouble()
        }
    }

    return {
        if (gamepad.backWasPressed()) {
            localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0 + 2.5, -PI/2).mirroredIf(isRed.get())
            targetPose.set(Pose(robotWidth/2.0, -72.0 + robotLength/2.0 + 2.5, -PI/2).mirroredIf(isRed.get()))
        }
        val dError = -getRelativeVelocity(localizer.pose, localizer.poseVel)
        processTurnTranslational(heading(dError), teleopTranslational(dError.vector()), localizer.pose, localizer.poseVel)
    }
}

fun getHeadingLockTeleop(getAngle: () -> Double?, gamepad: Gamepad, localizer: Localizer, teleopTranslational: (dError: Vector) -> Vector, isRed: Reference<Boolean>, targetPose: Reference<Pose>) = {
    if (gamepad.backWasPressed()) {
        localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0 + 2.0, -PI/2).mirroredIf(isRed.get())
        targetPose.set(Pose(robotWidth/2.0, -72.0 + robotLength/2.0 + 2.0, -PI/2).mirroredIf(isRed.get()))
    }

    val angle = getAngle() ?: localizer.poseVel.vector().let {if (it.length > 3.0) it.angle else localizer.heading}
    val translational = teleopTranslational(getRelativeVelocity(localizer.pose, localizer.poseVel).vector() * -1)
    val turn = PDLT(AngleUnit.normalizeRadians(angle - localizer.heading), -localizer.headingVel, hP, hD, kS, hT)

    processTurnTranslational(turn, translational, localizer.pose, localizer.poseVel)
}