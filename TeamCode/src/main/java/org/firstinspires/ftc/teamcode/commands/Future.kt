package org.firstinspires.ftc.teamcode.commands

class Future(val f: () -> Command, name: String = "Future"): OverrideButtonCommand(name) {
    private var command: Command? = null
    override fun run(): CommandResult {
        command = command ?: f()
        val result = command!!.update()
        when (result){
            CommandResult.Continue -> {}
            is CommandResult.End -> command = command!!.getDead(result)
        }
        return result

    }

    override fun getDead(reason: CommandResult.End): DeadCommand {
        val command = command
        return DeadCommand(
            reason,
            when (command) {
                null -> ArrayList()
                is DeadCommand -> arrayListOf(command)
                else -> arrayListOf(command.getDead(
                    CommandResult.End(Result.failure(Error("Parent ended early")))
                ))
            },
            name, selfCondense, uid
        )
    }
}