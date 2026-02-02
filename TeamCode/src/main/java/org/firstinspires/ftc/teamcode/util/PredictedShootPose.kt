package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.subsystems.drive.getStopPosition
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import kotlin.math.PI
import kotlin.math.pow

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
                    .let{Pair(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                    .let{Pair(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, robotLength/2))
                    .let{Pair(movementStart + it, movementEnd + it)}
            ) + getLaunchZoneIntersections(
                (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                    .let{Pair(movementStart + it, movementEnd + it)}
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