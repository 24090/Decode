package org.firstinspires.ftc.teamcode.util

import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.pow

class Polynomial(vararg val coefficients: Double) {
    val length = coefficients.size
    val degree = coefficients.size - 1
    fun evaluate(value: Double): Double{
        return coefficients.withIndex().sumOf { (i, coefficient) -> coefficient * value.pow(i) }
    }
    fun derivative() = Polynomial(
        *coefficients
            .withIndex()
            .map {(i,coefficient) -> coefficient*i}
            .subList(1, length)
            .toDoubleArray()
        )

    fun integral(C: Double = 0.0) = Polynomial(
        *coefficients
            .withIndex()
            .map {(i,coefficient) -> coefficient/(i+1)}
            .let { listOf(C).plus(it) }
            .toDoubleArray()
        )

    fun evaluateInterval(interval: Interval) =
        coefficients
            .withIndex()
            .map { (i, coefficient) -> interval.pow(i) * coefficient}
            .reduce(Interval::plus)

    fun minimize(interval: Interval): List<Double> {
        val possible = derivative()
            .solve(interval)
            .map { Pair(it, evaluate(it)) }
            .plus(Pair(interval.lower, evaluate(interval.lower)))
            .plus(Pair(interval.upper, evaluate(interval.upper)))
            .sortedBy { (_, b) -> b.absoluteValue }

        return possible.takeWhile { (_, b) -> (b - possible.first().second) <= 1e-5}.map { (a, _) -> a }
    }

    fun solve(interval: Interval, iterations: Int = 16): List<Double>{
        val out = evaluateInterval(interval)
        if (!out.contains(0.0)) {
            return listOf()
        }
        if (out.lower == out.upper) {
            return listOf(interval.lower)
        }
        val pivot = (interval.lower + interval.upper)/2
        if (iterations == 1) {
            return listOf(pivot)
        }
        return solve(
            Interval(interval.lower, pivot),
            iterations - 1,
        ) + solve(
            Interval(pivot, interval.upper),
            iterations - 1,
        )
    }

    operator fun plus(other: Double): Polynomial {
        return Polynomial(
            *DoubleArray(max(this.length, 1)){ i ->
                (if (i < this.length) coefficients[i] else 0.0) +
                (if (i < 1) other else 0.0)
            }
        )
    }

    operator fun plus(other: Polynomial): Polynomial {
        return Polynomial(
            *DoubleArray(max(this.length, other.length)){ i ->
                (if (i < this.length) coefficients[i] else 0.0) +
                (if (i < other.length) other.coefficients[i] else 0.0)
            }
        )
    }

    operator fun times(other: Double) = Polynomial(
        *DoubleArray(this.length * 1){ i -> this.coefficients[i] * other }
    )

    operator fun times(other: Polynomial): Polynomial {
        val newCoefficients = DoubleArray(this.degree + other.degree + 1)
        for (i in 0..<this.length) {
            for (j in 0..<other.length) {
                newCoefficients[i + j] += this.coefficients[i] * other.coefficients[j]
            }
        }
        return Polynomial(*newCoefficients)
    }

    fun pow(n: Int): Polynomial {
        var v = Polynomial(1.0)
        for (i in 1..n) {
            v = v * this
        }
        return v
    }
}