package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

fun Int.factorial() : Int = (1..this).reduce(Int::times)
/**
 * @return The number of ways to choose `r` unique elements from a set of `n` without replacement, ignoring order
 */
fun nCr(n: Int, r: Int) = n.factorial()/(r.factorial() * (n-r).factorial())
/**
 * Infix version of `nCr`
 * @see nCr
 */
infix fun Int.choose(r: Int) = nCr(this, r)

fun clamp(x: Double, min: Double, max: Double): Double {
    assert(min <= max)
    return min(max(x, min), max)
}

fun solvePolynomialIA(interval: Interval, iterations: Int = 50, vararg coefficients: Double, first: Boolean = true): List<Double>{
    val length = coefficients.size
    val out = coefficients.withIndex().map { (i, coefficient) ->  interval.pow(length - i - 1) * coefficient}.reduce(Interval::plus)
    if (!out.contains(0.0)) {
        if (first) {
            val closest = if (out.upper < 0.0) out.upper else out.lower
            println("Closest to zero: $closest")
            val newCoefficients = coefficients.dropLast(1).plus(coefficients.last() - closest).toDoubleArray()
            return solvePolynomialIA(interval, iterations, *newCoefficients)
        }
        return listOf()
    }
    val pivot = (interval.lower + interval.upper)/2
    if (iterations == 1) {
        return listOf(pivot)
    }
    return solvePolynomialIA(
        Interval(interval.lower, pivot),
        iterations - 1,
        *coefficients,
        first = false
    ) + solvePolynomialIA(
        Interval(pivot, interval.upper),
        iterations - 1,
        *coefficients,
        first = false
    )
}
fun solveQuarticNewton(a: Double, b: Double, c: Double, d: Double, e: Double, guess: Double): Double{
    val derivA: Double = 4*a
    val derivB: Double = 3*b
    val derivC: Double = 2*c
    val derivD: Double = 1*d
    var initGuess: Double = guess
    var i = 0
    while (i<100){
        val slope: Double = evalPolynomial(initGuess, derivA,derivB,derivC, derivD)
        val newGuess: Double = initGuess -(evalPolynomial(initGuess, a,b,c,d,e))/slope
        initGuess = newGuess
        i++
    }
    return initGuess
}
fun evalPolynomial(value: Double, vararg coefficients: Double): Double{
    val length = coefficients.size
    return coefficients.withIndex().map { (i, coefficient) ->  coefficient * value.pow(length - i - 1)}.sum()
}
fun derivPolynomial(vararg coefficients: Double): List<Double>{
    val degree = coefficients.size - 1
    return coefficients.withIndex().map {(i,coefficient) -> coefficient*(degree-i)}
}

fun findLineIntersection(lineA: Pair<Vector, Vector>, lineB: Pair<Vector, Vector>): Vector? {

    // i'm to lazy to handle vertical lines so adding 0.001 is the solution
    val lineA = if (lineA.first.x == lineA.second.x) {
        Pair(Vector.fromCartesian(lineA.first.x + 0.001, lineA.first.y), lineA.second)
    } else lineA

    val lineB = if (lineB.first.x == lineB.second.x) {
        Pair(Vector.fromCartesian(lineB.first.x + 0.001, lineB.first.y), lineB.second)
    } else lineB

    val slopeA = (lineA.first.y - lineA.second.y)/(lineA.first.x - lineA.second.x)
    val slopeB = (lineB.first.y - lineB.second.y)/(lineB.first.x - lineB.second.x)
    val interceptA = lineA.first.y - lineA.first.x * slopeA
    val interceptB = lineB.first.y - lineB.first.x * slopeB
    val pointInRange = { x: Double, a: Double, b: Double -> x>= min(a, b)&& x<= max(a, b)}
    val x = (interceptA - interceptB) / (slopeB - slopeA)
    if (pointInRange(x, lineA.first.x, lineA.second.x) && pointInRange(x, lineB.first.x, lineB.second.x)) {
        return Vector.fromCartesian(x, x * slopeA + interceptA)
    }

    if (slopeA == slopeB) {
        return (
            if (interceptB != interceptA) null
            else if (pointInRange(lineB.first.x, lineA.first.x, lineA.second.x)) lineB.first
            else if (pointInRange(lineB.second.x, lineA.first.x, lineA.second.x)) lineB.second
            else null
        )
    }
    return null

}
