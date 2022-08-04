package hr.unipu.event;

import hr.unipu.foodcomputer.FoodComputerCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * Holds list of listeners, adds new listener, sends event (food computer command) to
 * all listeners, with method from EventListener interface - onQueueMessage().
 */
public class EventManager {

    /**
     * The list with components to be notified of a new food computer Command received from Mosquitto.
     */
    private List<EventListener> eventListeners = new ArrayList<>();


    /**
     * Used by every component which wants to be notified of new events.
     *
     * @param eventListener {@link EventListener}
     */
    public void addListener(EventListener eventListener) {
        this.eventListeners.add(eventListener);
    }


    /**
     * Used by Mosquitto callback to forward a received messaged to all components in the application who were added
     * as a listener.
     *
     * @param foodComputerCommand {@link FoodComputerCommand}
     */
    public void sendEvent(FoodComputerCommand foodComputerCommand) {
        this.eventListeners.forEach(listener -> listener.onQueueMessage(foodComputerCommand));
    }

}
