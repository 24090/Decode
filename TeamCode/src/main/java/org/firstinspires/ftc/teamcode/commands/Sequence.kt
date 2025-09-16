package org.firstinspires.ftc.teamcode.commands

class Sequence(vararg commands: Command): OverrideButtonCommand("Sequence", true){
    private val pastCommands: ArrayList<DeadCommand> = ArrayList()
    private val futureCommands:  ArrayList<Command> = ArrayList()
    private var currentCommand: Command?
    init {
        commands.toCollection(futureCommands)
        currentCommand = futureCommands.removeAt(0)
    }

    override fun getFuture(): FutureCommand{
        val commands: ArrayList<Command> = pastCommands.clone() as ArrayList<Command>
        currentCommand?.let { commands.add(it.getFuture()) }
        for (command in futureCommands) {
            commands.add(command.getFuture())
        }
        return FutureCommand(commands, name, selfCondense, uid)
    }

    override fun getDead(reason: CommandResult.End): DeadCommand{
        val parentEndedEarly = CommandResult.End(Result.failure(Error("Parent ended early")))
        currentCommand?.let { pastCommands.add(it.getDead(parentEndedEarly)) }
        for (command in futureCommands) {
            pastCommands.add(command.getDead(parentEndedEarly))
        }
        return DeadCommand(reason, pastCommands, name, selfCondense, uid)
    }

    override fun run(): CommandResult {
        // uses local val so only one null-check necessary per loop
        val currentCommand = currentCommand
            ?: return CommandResult.End(Result.success("No Commands Left"))

        // show dead past commands
        for (command in pastCommands) {
            command.update()
        }
        val result = currentCommand.update()
        // show future commands
        for (command in futureCommands) {
            command.getFuture().update()
        }
        when (result) {
            CommandResult.Continue -> return CommandResult.Continue
            is CommandResult.End -> {
                pastCommands.add(currentCommand.getDead(result))
                if (result.result.isFailure){
                    return CommandResult.End(Result.failure(Error("Child error")))
                }
                this.currentCommand = try {
                    futureCommands.removeAt(0)
                } catch (e: IndexOutOfBoundsException) {
                    null
                }
                return CommandResult.Continue
            }
        }
    }
}