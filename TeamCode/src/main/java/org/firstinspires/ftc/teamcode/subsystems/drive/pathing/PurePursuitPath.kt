package org.firstinspires.ftc.teamcode.subsystems.drive.pathing

import android.util.Log
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.HeadingBehaviour
import org.firstinspires.ftc.teamcode.util.Line
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.abs

class PurePursuitPath(val poses: List<Pose>, val headingBehaviours: List<HeadingBehaviour>, val maxRadii: List<Double>) {
    val points = poses.map(Pose::vector)
    var lastT = 0.0
    val lines = points
        .subList(0, points.size - 1)
        .zip(points.subList(1, points.size))
        .map { Line(it.first, it.second) }

    fun adjustT(t: Double): Double{
        val t = clamp(t, 0.0, lines.size.toDouble() - 1e-10)
        return (0..t.toInt()).sumOf { lines[it].length } + lines[t.toInt()].length * (t%1)
    }

    fun getFollowPoint(currentPosition: Vector, tThresh: Double, updateLastT: Boolean = true): Pose {
        if ((lastT > (lines.size-1))&&((currentPosition - points.last()).length < maxRadii[lastT.toInt()] * 2.0)){
            return poses.last()
        }
        val lastAdjustedT = adjustT(lastT)
        val (t, followPoint) = lines
            .mapIndexed { index, line ->
                line.circleIntersection(
                    currentPosition,
                    maxRadii[lastT.toInt()]
                ).map { Pair(
                    index.toDouble() + line.getTvalue(it),
                    it
                )}
            }
            .reduce{ a, b -> a+b }
            .filter { abs(adjustT(it.first) - lastAdjustedT) <= tThresh }
            .reduceOrNull{ a, b -> if(a.first > b.first) a else b }
            ?: Pair(lastT, getPosition(lastT))
        val headingBehaviour = headingBehaviours[t.toInt()]
        val heading = when (headingBehaviour){
            is HeadingBehaviour.Tangent -> AngleUnit.normalizeRadians((followPoint - currentPosition).angle + headingBehaviour.angle)
            HeadingBehaviour.Snap -> poses[t.toInt() + 1].heading
            HeadingBehaviour.Interpolate -> poses[t.toInt()].heading * (1-(t%1)) + poses[t.toInt()+1].heading * (t%1)
        }

        if (updateLastT) {lastT = t}

        return Pose(followPoint.x, followPoint.y, heading)
    }

    fun getPosition(t: Double) = lines[t.toInt()].getPoint(t%1)
}