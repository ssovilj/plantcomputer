package hr.unipu.event;

import hr.unipu.plantcomputer.PlantComputerCommand;

public interface EventListener {

    /**
     * Whenever a new {@link PlantComputerCommand} is received from Mosquitto, all listeners will be notified, so they can handle
     * it for their own use.
     *
     * @param plantComputerCommand {@link PlantComputerCommand}
     */
    void onQueueMessage(PlantComputerCommand plantComputerCommand);

}
