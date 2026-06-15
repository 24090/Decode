package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.util.timeSeconds

class Sleep(val waitSeconds: Double, name: String = "Sleep"): OverrideButtonCommand(name) {
    override fun nextInstant() = when (run()){
        CommandResult.Continue -> false
        is CommandResult.End -> true
    }
    private var targetTime: Double? = null

    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = super.getButtons()
        return buttons
    }

    override fun run(): CommandResult {
        if (targetTime == null) {
            targetTime = timeSeconds() + waitSeconds
        }
        return if (timeSeconds() >= targetTime!!) {
            CommandResult.End(Result.success("End condition satisfied"))
        } else {
            CommandResult.Continue
        }
    }
}