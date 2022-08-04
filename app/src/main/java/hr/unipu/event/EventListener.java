package hr.unipu.event;

import hr.unipu.foodcomputer.FoodComputerCommand;

public interface EventListener {

    /**
     * Whenever a new {@link FoodComputerCommand} is received from Mosquitto, all listeners will be notified, so they can handle
     * it for their own use.
     *
     * @param foodComputerCommand {@link FoodComputerCommand}
     */
    void onQueueMessage(FoodComputerCommand foodComputerCommand);

}
