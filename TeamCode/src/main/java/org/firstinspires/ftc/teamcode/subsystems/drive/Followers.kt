package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyT
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getHeadingVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.BezierCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

fun getTeleopFollower(gamepad: Gamepad, localizer: Localizer, isRed: () -> Boolean): () -> DriveVectors {
    var lastLockTranslational = false
    var lastLockHeading = false
    var targetPose = Pose(0.0, 0.0, 0.0)
    val isLockHeading = {
        gamepad.right_stick_x.toDouble() == 0.0 //&& (drive.localizer.headingVel < 0.02 || lastLockHeading)
    }
    val isLockTranslational = {
        gamepad.left_stick_x.toDouble() == 0.0 && gamepad.left_stick_y.toDouble() == 0.0// && ((drive.localizer.xVel < 0.2 && drive.localizer.yVel < 0.2) || lastLockTranslational)
    }
    val heading = { error: Pose, dError: Pose ->
        val lockHeading = isLockHeading()
        if (lockHeading){
            if (!lastLockHeading){
                targetPose.heading = localizer.heading
            }
            lastLockHeading = true
            PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
        } else {
            lastLockHeading = false
            -gamepad.right_stick_x.toDouble()
        }
    }

    val translational = { error: Pose, dError: Pose ->
        val lockTranslational = isLockTranslational()
        if (lockTranslational) {
            if (!lastLockTranslational) {
                val maxAccel = if (dError.x < 0) tipAccelForward else tipAccelBackward
                val t = (-dError.x)/maxAccel
                val targetPosition = localizer.pose.vector() + Vector.fromPolar(localizer.heading, maxAccel * t.pow(2)/2 + (-dError.x) * t)
                targetPose.x = targetPosition.x
                targetPose.y = targetPosition.y
            }
            lastLockTranslational = true
            PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
        } else {
            var v = Vector.fromCartesian(-gamepad.left_stick_x.toDouble(), gamepad.left_stick_y.toDouble())
            v = v * v.length * 1.67
            if (!isRed()) v = v.rotated(PI)
            lastLockTranslational = false
            v.rotated(-localizer.heading)
        }
    }

    return {
        if (gamepad.backWasPressed()) {
            localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed())
            targetPose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed())
        }
        val error = getRelativePose(localizer.pose, targetPose)
        val dError = -getRelativeVelocity(localizer.pose, localizer.poseVel)
        processTurnTranslational(heading(error, dError), translational(error, dError), localizer.pose, localizer.poseVel)
    }
}

fun followCurve(curve: BezierCurve, pose: Pose, velocity: Pose): DriveVectors{
    // 1. corrective position/heading
    // 2. static friction feedforward
    // 3. accelvelstuff

    val t = curve.closestT(pose.vector())
    val curvePose = curve.getPose(t)
    val curveVelocity = curve.getVelocity(t)
    val curveAccel = curve.getAcceleration(t)
    val corrective = pointToPoint(pose, velocity, curvePose, false)
    val static = curveVelocity.vector().normalized() * kS
    val goodVelocityLength = max((velocity.vector() dot curveVelocity.vector().norm()), 0.0)
    val accelvel = (Vector.fromPolar(
        angle = curveAccel.vector().angle,
        length = goodVelocityLength * (curveAccel.vector().length / curveVelocity.vector().length)
    ) + Vector.fromPolar(
        angle = curveVelocity.vector().angle,
        goodVelocityLength,
    )) * 2.0

    return corrective
        .addWithoutPriority(getTranslationalVectors(static.x, static.y), controlHubVoltage / 14.0)
        .tipCorrected(
            tipAccelBackward, tipAccelForward,
            kS, kV, kA,
            getRelativeVelocity(pose, velocity).x
        )

}

fun getPointToPoint(targetPose: Pose, localizer: Localizer) = {
        pointToPoint(localizer.pose, localizer.poseVel, targetPose)
    }

fun pointToPoint(currentPose: Pose, currentVelocity: Pose, targetPose: Pose, full: Boolean = true): DriveVectors {
    val error = getRelativePose(currentPose, targetPose)
    val dError = -getRelativeVelocity(currentPose, currentVelocity)
    val turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
    val translational =
        PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
    val drive = if (translational.x.isNaN()) 0.0 else translational.x
    val strafe = if (translational.y.isNaN()) 0.0 else translational.y

    return if (!full) {
        getHeadingVectors(turn) + getTranslationalVectors(drive, strafe)
    } else {
        processTurnDriveStrafe(turn, drive, strafe, currentPose, currentVelocity)
    }
}

fun processTurnTranslational(turn: Double, translational: Vector, pose: Pose, velocity: Pose) =
    processTurnDriveStrafe(turn, translational.x, translational.y, pose, velocity)

fun processTurnDriveStrafe(turn: Double, drive: Double, strafe: Double, pose: Pose, velocity: Pose) =
    getHeadingVectors(turn)
        .addWithoutPriority(getTranslationalVectors(drive, strafe), controlHubVoltage / 14.0)
        .tipCorrected(
            tipAccelBackward, tipAccelForward,
            kS, kV, kA,
            getRelativeVelocity(pose, velocity).x
        )

fun DriveVectors.tipCorrected(minAccel: Double, maxAccel: Double, kS: Double, kV: Double, kA: Double, velocity: Double): DriveVectors {
    val minPower: Double = minAccel * kA + velocity * kV + kS * sign(velocity)
    val maxPower: Double = maxAccel * kA + velocity * kV + kS * sign(velocity)
    val power = (left.x + right.x)/2.0

    println("minPower: $minPower, maxPower: $maxPower, power: $power")

    return if (power > maxPower) {
        DriveVectors(left * (maxPower/power), right * (maxPower/power))
    } else if (power < minPower) {
        DriveVectors(left * (minPower/power), right * (minPower/power))
    } else {
        this
    }
}