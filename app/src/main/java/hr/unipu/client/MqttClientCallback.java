package hr.unipu.client;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import hr.unipu.event.EventManager;
import hr.unipu.foodcomputer.FoodComputerCommand;
import javafx.application.Platform;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Implementation of MqttCallback interface methods: connectionLost(), deliveryComplete(), messageArrived().
 */
public class MqttClientCallback implements MqttCallback {

    final EventManager eventManager;


    public MqttClientCallback(EventManager eventManager) {
        this.eventManager = eventManager;
    }


    @Override
    public void connectionLost(Throwable throwable) {
        System.out.println("Connection to MQTT broker lost!");
    }


    @Override
    public void messageArrived(String mqttTopic, MqttMessage mqttMessage) {
        String jsonMessage = new String(mqttMessage.getPayload());

        if (mqttTopic.equals("foodComputerCommand")) {
            System.out.println("Filtered MQTT topic: \"" + mqttTopic + "\".");
            System.out.println("Message received:\n\t" + jsonMessage);
            Platform.runLater(() -> {
                System.out.println("Forwarding MQTT message (of actuators) to all listeners.");
                parseJsonMessage2Commands(jsonMessage);

            });

        } else if (mqttTopic.equals("foodComputerState")) {
            System.out.println("Filtered MQTT topic: \"" + mqttTopic + "\".");
            System.out.println("Message received:\n\t" + jsonMessage);
            Platform.runLater(() -> {
                System.out.println("Forwarding MQTT message (from sensors) to all listeners.");
                parseJsonMessage2Readings(jsonMessage);
            });

        }


    }

    /**
     * Parse jsonMessage to commands.
     * @param jsonMessage
     */
    private void parseJsonMessage2Commands(String jsonMessage) {
        ObjectMapper objectMapper = new ObjectMapper();
        //TypeFactory typeFactory = objectMapper.getTypeFactory();
        try {
            //List<FoodComputerCommand> parsedListActuatorCommands = objectMapper.readValue(jsonMessage, typeFactory.constructCollectionType(List.class, FoodComputerCommand.class));
            FoodComputerCommand[] parsedListActuatorCommands = objectMapper.readValue(jsonMessage, FoodComputerCommand[].class);
            for (FoodComputerCommand parsedListActuatorCommand : parsedListActuatorCommands) {
                //System.out.println(parsedListActuatorCommand.toStringCommand());
                eventManager.sendEvent(parsedListActuatorCommand);
            }
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }


    /**
     * Parse jsonMessage to sensors' readings.
     * @param jsonMessage
     */
    private void parseJsonMessage2Readings(String jsonMessage) {
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            FoodComputerCommand[] parsedListSensorReadings = objectMapper.readValue(jsonMessage, FoodComputerCommand[].class);
            for (FoodComputerCommand parsedListActuatorReading : parsedListSensorReadings) {
                //System.out.println(parsedListActuatorReading.toStringCommand());
                eventManager.sendEvent(parsedListActuatorReading);
            }
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        System.out.println("Delivery complete.");
    }

}