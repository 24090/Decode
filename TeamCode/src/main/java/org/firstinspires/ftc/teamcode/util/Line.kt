package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.RobotLog.v
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

class Line(val start: Vector, val end: Vector) {
    val length
        get() = (start - end).length
    fun getTvalue(point: Vector): Double{
        if (abs(start.x - end.x)< abs(start.y - end.y)){
            if (start.y == end.y){
                return 1.0
            }
            return (point.y - start.y)/(end.y - start.y)
        } else if (start.x == end.x){
            return 1.0
        }
        return (point.x - start.x)/(end.x - start.x)
    }

    fun projectPoint(point: Vector): Vector {
        val result = (end - start).projection(point) + start
        return if (getTvalue(result) > 1) {
            end
        } else if (getTvalue(result) < 0) {
            start
        } else {
            result
        }

    }

    fun getPoint(t: Double): Vector {
        return start * (1-t) + end * t
    }

    fun lineIntersection(other: Line): Vector?{
        // i'm to lazy to handle vertical lines so adding 0.001 is the solution
        val lineA = if (this.start.x == this.end.x) {
            Line(Vector.fromCartesian(this.start.x + 0.001, this.start.y), this.end)
        } else this

        val lineB = if (other.start.x == other.end.x) {
            Line(Vector.fromCartesian(other.start.x + 0.001, other.start.y), other.end)
        } else other

        val slopeA = (lineA.start.y - lineA.end.y)/(lineA.start.x - lineA.end.x)
        val slopeB = (lineB.start.y - lineB.end.y)/(lineB.start.x - lineB.end.x)
        val interceptA = lineA.start.y - lineA.start.x * slopeA
        val interceptB = lineB.start.y - lineB.start.x * slopeB
        val pointInRange = { x: Double, a: Double, b: Double -> x>= min(a, b)&& x<= max(a, b)}
        val x = (interceptA - interceptB) / (slopeB - slopeA)
        if (pointInRange(x, lineA.start.x, lineA.end.x) && pointInRange(x, lineB.start.x, lineB.end.x)) {
            return Vector.fromCartesian(x, x * slopeA + interceptA)
        }

        if (slopeA == slopeB) {
            return (
                    if (interceptB != interceptA) null
                    else if (pointInRange(lineB.start.x, lineA.start.x, lineA.end.x)) lineB.start
                    else if (pointInRange(lineB.end.x, lineA.start.x, lineA.end.x)) lineB.end
                    else null
                    )
        }
        return null
    }

    fun circleIntersection(center: Vector, radius: Double): List<Vector> =
        Line(this.start - center, this.end - center).circleIntersection(radius).map { it + center }

    fun circleIntersection(radius: Double): List<Vector>{
        val dx = end.x - start.x
        val dy = end.y - start.y
        val dr = sqrt(dx * dx + dy * dy)
        val D = start.x * end.y - end.x * start.y
        val discriminant = radius*radius * dr*dr - D*D
        val sgn = { v: Double ->
            if (v >= 0){
                1.0
            } else {
                -1.0
            }
        }
        return if (discriminant < 0.0){
            listOf()
        } else if (discriminant == 0.0){
            println("ZERO!")
            listOf(Vector.fromCartesian(
                (D * dy)/(dr*dr),
                -(D * dx)/(dr*dr)
            ))
        } else {
            val a = Vector.fromCartesian(
                ((D * dy) + sgn(dy)*dx*sqrt(discriminant))/(dr*dr),
                (-(D * dx) +   abs(dy)*sqrt(discriminant))/(dr*dr)
            )
            val b = Vector.fromCartesian(
                ((D * dy) - sgn(dy)*dx*sqrt(discriminant))/(dr*dr),
                (-(D * dx) -   abs(dy)*sqrt(discriminant))/(dr*dr)
            )
            listOf(a, b)
        }.filter { point -> getTvalue(point).let { 0 < it && it < 1 } }
    }

    override fun toString() = "[${this.start} -> ${this.end}]"
}