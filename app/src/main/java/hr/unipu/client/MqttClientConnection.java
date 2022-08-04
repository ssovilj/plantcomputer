package hr.unipu.client;

import hr.unipu.event.EventManager;
import hr.unipu.foodcomputer.FoodComputerCommand;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Initialize connection with MQTT server, connect, subscribe to MQTT topic,
 * and set callback method (with forwarding of event manager).
 */
public class MqttClientConnection {

    private MqttClient mqttClient;
    private String mqttTopic;
    private final EventManager eventManager;


    public MqttClientConnection(String mqttServerURI, EventManager eventManager) {
        this.eventManager = eventManager;
        if (!this.initConnection(mqttServerURI)) {
            System.err.println("Initializing connection failed.");
            return;
        }
        this.connect();
    }


    private boolean initConnection(String mqttServerURI) {
        try {
            this.mqttClient = new MqttClient("tcp://" + mqttServerURI + ":1883", MqttClient.generateClientId());
            System.out.println("Mosquitto client initialized.");
            return true;
        } catch (MqttException ex) {
            System.err.println("MqttException: " + ex.getMessage());
        }
        return false;
    }


    private void connect() {
        try {
            this.mqttClient.connect();
            System.out.println("Connected to Mosquitto client.");
        } catch (MqttException ex) {
            System.err.println("MqttException: " + ex.getMessage());
        }
    }


    public void subscribe(String mqttTopic) {
        this.mqttTopic = mqttTopic;
        try {
            this.mqttClient.subscribe(this.mqttTopic);
            this.mqttClient.setCallback(new MqttClientCallback(eventManager));
            System.out.println("Subscribed to Mqtt topic: " + mqttTopic);
        } catch (MqttException ex) {
            System.err.println("Error while subscribing: " + ex.getMessage());
        }
    }


    public void sendMessage(FoodComputerCommand foodComputerCommand, String mqttTopic) {
        this.mqttTopic = mqttTopic;

        //this.sendMessage(foodComputerCommand.toStringCommand());      // Sending old command e.g. 100:APLN 1:on
        this.sendMessage(foodComputerCommand.toJsonCommand(), mqttTopic);          // Sending JSON command.
    }


    public void sendMessage(String messageText, String mqttTopic) {
        this.mqttTopic = mqttTopic;

        if (!this.mqttClient.isConnected()) {
            System.err.println("The queue client is not connected!");
            this.connect();
        }

        MqttMessage message = new MqttMessage();
        message.setPayload(messageText.getBytes());

        try {
            this.mqttClient.publish(this.mqttTopic, message);
        } catch (MqttException ex) {
            System.err.println("MqttException: " + ex.getMessage());
        }
    }


    public void disconnect() {
        try {
            this.mqttClient.disconnect();
        } catch (MqttException ex) {
            System.err.println("MqttException: " + ex.getMessage());
        }
    }


    public MqttClient getMqttClient() {
        return mqttClient;
    }

}