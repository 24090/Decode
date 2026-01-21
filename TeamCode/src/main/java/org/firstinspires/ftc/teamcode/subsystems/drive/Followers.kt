package org.firstinspires.ftc.teamcode.subsystems.drive

import android.util.Log
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
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
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.BezierCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import org.firstinspires.ftc.teamcode.util.Reference
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

fun getStopPosition(pose: Pose, relativeVelocity: Vector): Vector{
    val maxAccel = if (relativeVelocity.x > 0) tipAccelForward else tipAccelBackward
    val t = relativeVelocity.x/maxAccel
    return pose.vector() + Vector.fromPolar(pose.heading, maxAccel * t.pow(2)/2 + relativeVelocity.x * t)
}
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
                targetPose.set(getStopPosition(localizer.pose, -dError).let{
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
            localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed.get())
            targetPose.set(Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed.get()))
        }
        val dError = -getRelativeVelocity(localizer.pose, localizer.poseVel)
        processTurnTranslational(heading(dError), teleopTranslational(dError.vector()), localizer.pose, localizer.poseVel)
    }
}

fun getFollowCurve(curve: BezierCurve, localizer: Localizer) = { followCurve(curve, localizer.pose, localizer.poseVel) }
fun followCurve(curve: BezierCurve, pose: Pose, velocity: Pose): DriveVectors{
    // 1. corrective position/heading
    // 2. static friction feedforward
    // 3. accelvelstuff

    val t = curve.closestT(pose.vector())
    println(t)
    val curvePose = curve.getPose(t)
    val curveVelocity = curve.getVelocity(t).vector()
    val curveAccel = curve.getAcceleration(t).vector()
    val corrective = pointToPoint(pose, velocity, curvePose, false)
    val static = curveVelocity.normalized() * (kS + 0.4)
    val goodVelocityLength = max((velocity.vector() dot curveVelocity.norm()), 0.0)
    val accelvel = (Vector.fromPolar(
        angle = curveAccel.angle,
        length = goodVelocityLength * (curveAccel.length / curveVelocity.length.let { if (it == 0.0) 0.001 else it} )
    ) + Vector.fromPolar(
        angle = curveVelocity.angle,
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
fun minStopDistance(heading: Double, errorAngle: Double, velocity: Pose, minAccelX: Double, maxAccelX: Double): Double{
    val errorNorm = Vector.fromPolar(errorAngle, 1.0)
    val v = errorNorm.scalarProjection(velocity.vector())
    val vX = Vector.fromPose(velocity).rotated(-heading).x
    val uncorrectedPower = DriveVectors
        .fromTranslation(errorNorm * -100)
        .trimmed(controlHubVoltage / 14.0)
        .left.length


    val minAccel = errorNorm.scalarInverseProjection(Vector.fromPolar(0.0, minAccelX))
    val minPower = minAccel * kA + vX * kV + kS * sign(v)

    val maxAccel = errorNorm.scalarInverseProjection(Vector.fromPolar(0.0, maxAccelX))
    val maxPower = maxAccel * kA + vX * kV + kS * sign(v)


    val constantAccel = if (minPower > uncorrectedPower) minAccel else if (maxPower < uncorrectedPower) maxAccel else uncorrectedPower
    println("minAccel: $minAccelX")
    println("maxAccel: $minAccelX")
    println("constantAccel: $constantAccel")
    val t = max(
        (uncorrectedPower - (constantAccel * kA + v * kV + kS * sign(v)))/(constantAccel * kV),
        0.0
    )
    val s = 0 + t * v + t.pow(2)/2 * constantAccel
    val newVelocity = v + constantAccel * t
    return s + minStopDistanceWithoutTipCorrection(errorAngle, newVelocity)
}

fun minStopDistanceWithoutTipCorrection(errorAngle: Double, velocity: Double): Double{
    val maxStopPower = DriveVectors
        .fromTranslation(Vector.fromPolar(errorAngle, -100.0))
        .trimmed(controlHubVoltage / 14.0).left.length
    val initialAcceleration = (maxStopPower - kS * velocity.absoluteValue - kV * velocity)/kA
    val k1 = (kA/kV) * initialAcceleration
    return  kA/kV * ((velocity - k1)*ln((k1 - velocity)/k1) - velocity)
}

fun getScaryPathing(targetPose: Pose, localizer: Localizer) = { scaryPathing(localizer.pose, localizer.poseVel, targetPose) }
fun scaryPathing(pose: Pose, velocity: Pose, targetPose: Pose): DriveVectors{
    // if requiredDeaccel < maxDeaccel: go at max accel
    // when requiredDeaccel = maxDeaccel: go at max deaccel
    val error = (targetPose - pose)

    if (error.vector().length < 5.0){
        return pointToPoint(pose, velocity, targetPose, true)
    }

    val minStopDistance = minStopDistance(pose.heading, error.vector().angle, velocity, tipAccelBackward, tipAccelForward)
    val tooScary = (minStopDistance) > error.vector().length

    return if (tooScary){
        android.util.Log.w("#scarypathing", "too scary!!! $error")
        println("tooScary!!! $error")
        processTurnTranslational(0.0, error.vector().norm() * -100, pose, velocity)
    } else {
        Log.w("#scarypathing", "not scary :) $error, $minStopDistance")
        processTurnTranslational(0.0, error.vector().norm() * 100, pose, velocity)
    }
}

fun getPointToPoint(targetPose: Reference<Pose>, localizer: Localizer) = { pointToPoint(localizer.pose, localizer.poseVel, targetPose.get()) }
fun pointToPoint(pose: Pose, velocity: Pose, targetPose: Pose, full: Boolean = true): DriveVectors {
    val error = getRelativePose(pose, targetPose)
    val dError = -getRelativeVelocity(pose, velocity)
    val turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
    val translational =
        PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
    val drive = if (translational.x.isNaN()) 0.0 else translational.x
    val strafe = if (translational.y.isNaN()) 0.0 else translational.y
    Log.i("errorh", "${error.heading}")
    return if (!full) {
        DriveVectors.fromRotation(turn) + DriveVectors.fromTranslation(drive, strafe)
    } else {
        processTurnDriveStrafe(turn, drive, strafe, pose, velocity)
    }
}


fun getMoveShootPointToPoint(targetPose: Pose, localizer: Localizer, getAngle: () -> Double?) = { moveShootPointToPoint(localizer.pose, localizer.poseVel, targetPose, getAngle()) }

fun moveShootPointToPoint(pose: Pose, velocity: Pose, targetPose: Pose, angle: Double?): DriveVectors {

    val targetPose = Pose(targetPose.x, targetPose.y, angle ?: targetPose.heading)
    val error = getRelativePose(pose, targetPose)
    val dError = -getRelativeVelocity(pose, velocity)
    val turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
    val translational =
        PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
    val drive = if (translational.x.isNaN()) 0.0 else translational.x
    val strafe = if (translational.y.isNaN()) 0.0 else translational.y

    return processTurnDriveStrafe(turn, drive, strafe, pose, velocity)
}


fun processTurnTranslational(turn: Double, translational: Vector, pose: Pose, velocity: Pose) =
    processTurnDriveStrafe(turn, translational.x, translational.y, pose, velocity)

fun processTurnDriveStrafe(turn: Double, drive: Double, strafe: Double, pose: Pose, velocity: Pose) =
    DriveVectors.fromRotation(turn)
        .addWithoutPriority(DriveVectors.fromTranslation(drive, strafe), controlHubVoltage / 14.0)
        .tipCorrected(
            tipAccelBackward, tipAccelForward,
            kS, kV, kA,
            getRelativeVelocity(pose, velocity).x
        )

fun DriveVectors.strafeAccelCorrected(minAccel: Double, maxAccel: Double, kS: Double, kV: Double, kA: Double, velocity: Double): DriveVectors {
    val minPower: Double = minAccel * kA + velocity * kV + kS * sign(velocity)
    val maxPower: Double = maxAccel * kA + velocity * kV + kS * sign(velocity)
    val power = (left.y + right.y)/2.0

    return if (power > maxPower) {
        DriveVectors(left * (maxPower/power), right * (maxPower/power))
    } else if (power < minPower) {
        DriveVectors(left * (minPower/power), right * (minPower/power))
    } else {
        this
    }
}

fun DriveVectors.tipCorrected(minAccel: Double, maxAccel: Double, kS: Double, kV: Double, kA: Double, velocity: Double): DriveVectors {
    val minPower: Double = minAccel * kA + velocity * kV + kS * sign(velocity)
    val maxPower: Double = maxAccel * kA + velocity * kV + kS * sign(velocity)
    val power = (left.x + right.x)/2.0

    return if (power > maxPower) {
        DriveVectors(left * (maxPower/power), right * (maxPower/power))
    } else if (power < minPower) {
        DriveVectors(left * (minPower/power), right * (minPower/power))
    } else {
        this
    }
}

fun getHeadingLockTeleop(getAngle: () -> Double?, gamepad: Gamepad, localizer: Localizer, teleopTranslational: (dError: Vector) -> Vector, isRed: Reference<Boolean>, targetPose: Reference<Pose>) = {
    if (gamepad.backWasPressed()) {
        localizer.pose = Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed.get())
        targetPose.set(Pose(robotWidth/2.0, -72.0 + robotLength/2.0, -PI/2).mirroredIf(isRed.get()))
    }

    val angle = getAngle()
    val translational = teleopTranslational(getRelativeVelocity(localizer.pose, localizer.poseVel).vector() * -1)
    val turn = if (angle == null){
        0.0
    } else {
        PDLT(AngleUnit.normalizeRadians(angle - localizer.heading), -localizer.headingVel, hP, hD, kS, hT)
    }

    DriveVectors
        .fromRotation(turn)
        .addWithPriority (
            DriveVectors
                .fromTranslation(translational)
                .tipCorrected(
                    tipAccelBackward, tipAccelForward,
                    kS, kV, kA,
                    getRelativeVelocity(localizer.pose, localizer.poseVel).x
                ),
            controlHubVoltage / 14.0
        )
}