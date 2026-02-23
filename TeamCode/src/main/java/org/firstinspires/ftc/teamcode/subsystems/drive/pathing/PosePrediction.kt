package org.firstinspires.ftc.teamcode.subsystems.drive.pathing

import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import org.firstinspires.ftc.teamcode.util.Line
import org.firstinspires.ftc.teamcode.util.getLaunchZoneIntersections
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

fun minStopDistance(heading: Double, errorAngle: Double, velocity: Pose, minAccelX: Double, maxAccelX: Double): Double{
    val errorNorm = Vector.fromPolar(errorAngle, 1.0)
    val v = errorNorm.scalarProjection(velocity.vector())
    val vX = Vector.fromPose(velocity).rotated(-heading).x
    val uncorrectedPower = DriveVectors
        .fromTranslation(errorNorm * -100)
        .trimmed(controlHubVoltage / 14.0)
        .left.length

    val uncorrectedAccel = (uncorrectedPower - kV * v - kS)/kA

    val minAccel = errorNorm.scalarInverseProjection(Vector.fromPolar(0.0, minAccelX))
    val minPower = minAccel * kA + vX * kV + kS * sign(v)

    val maxAccel = errorNorm.scalarInverseProjection(Vector.fromPolar(0.0, maxAccelX))
    val maxPower = maxAccel * kA + vX * kV + kS * sign(v)


    val constantAccel = if (minPower > uncorrectedPower) minAccel else if (maxPower < uncorrectedPower) maxAccel else uncorrectedAccel
    println("minAccel: $minAccelX")
    println("maxAccel: $maxAccelX")
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

fun getStopPosition(pose: Pose, velocity: Pose): Vector{
    return pose.vector() + velocity.vector().norm() * minStopDistance(pose.heading, velocity.vector().angle, velocity, tipAccelBackward, tipAccelForward)
}

fun predictedShootPosition(minimumTime: Double, currentPose: Pose, fieldVelocity: Pose, estimatedAcceleration: Vector): Vector{
    val acceleration: Pose = estimatedAcceleration.let { Pose(it.x, it.y, 0.0) }
    val effectivePose = (currentPose + fieldVelocity * minimumTime + acceleration * minimumTime.pow(2) * 0.5)
    // Start turning/spinning up 2 seconds ahead
    val movementStart = effectivePose.vector()
    val movementEnd = (currentPose + fieldVelocity * 2.0 + acceleration * 2.0).vector()
    val closestIntersection: Vector? = if (inLaunchZone(effectivePose)) {
        effectivePose.vector()
    } else { (
            getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, robotLength/2))
                    .let{Line(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                    .let{Line(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, robotLength/2))
                    .let{Line(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                    .let{Line(movementStart + it, movementEnd + it)}
            )
            ).reduceOrNull { a, b -> if ((a - effectivePose.vector()).length < (b - effectivePose.vector()).length) a else b } }
    return closestIntersection?.let{ intersection ->
        val intersectionPose = Pose(intersection.x, intersection.y, currentPose.heading)
        val t  =
            if (fieldVelocity.x != 0.0) (closestIntersection - movementStart).x/fieldVelocity.x
            else if (fieldVelocity.y != 0.0) (closestIntersection - movementStart).y/fieldVelocity.y
            else 0.0
        getStopPosition(intersectionPose, fieldVelocity + acceleration * t)
    } ?: movementEnd

}