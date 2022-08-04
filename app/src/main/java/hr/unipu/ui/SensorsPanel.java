package hr.unipu.ui;

import hr.unipu.client.MqttClientConnection;
import hr.unipu.event.EventListener;
import hr.unipu.foodcomputer.FoodComputerCommand;
import hr.unipu.foodcomputer.FoodComputerAction;
import eu.hansolo.fx.colorselector.ColorSelector;
import javafx.geometry.HPos;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;

public class SensorsPanel extends HBox implements EventListener {

    private final MqttClientConnection mqttClientConnection;

    private final ColorSelector colorSelector1;
    private final ColorSelector colorSelector2;

    private final ToggleButton btStatic;
    private final ToggleButton btStaticFade;
    private final ToggleButton btBlinking;
    private final ToggleButton btRunning;
    private final ToggleButton btStaticRainbow;
    private final ToggleButton btFadingRainbow;
    private final ToggleButton btWhite;
    private final ToggleButton btClear;

    private final Slider slider;

    private FoodComputerAction selectedFoodComputerAction;

    private boolean blockSending = false;

    /**
     * Construct the UI.
     */
    SensorsPanel(MqttClientConnection mqttClientConnection) {
        this.mqttClientConnection = mqttClientConnection;

        this.setSpacing(25);

        VBox colorSelectors = new VBox();
        colorSelectors.setSpacing(25);
        this.getChildren().add(colorSelectors);

        this.colorSelector1 = new ColorSelector();
        this.colorSelector1.setPrefSize(250, 250);
        this.colorSelector1.selectedColorProperty().addListener(e -> this.sendMessage());
        this.colorSelector1.setSelectedColor(Color.BLUE);
        colorSelectors.getChildren().add(this.colorSelector1);

        this.colorSelector2 = new ColorSelector();
        this.colorSelector2.setPrefSize(250, 250);
        this.colorSelector2.selectedColorProperty().addListener(e -> this.sendMessage());
        this.colorSelector2.setSelectedColor(Color.RED);
        colorSelectors.getChildren().add(this.colorSelector2);

        GridPane effectButtons = new GridPane();
        effectButtons.setHgap(10);
        effectButtons.setVgap(10);
        this.getChildren().add(effectButtons);

        this.btStatic = new ToggleButton("Fixed");
        this.btStatic.getStyleClass().add("ledButton");
//        this.btStatic.setOnAction(e -> this.setEffect(FoodComputerActions.STATIC));
        effectButtons.add(this.btStatic, 0, 0);

        this.btStaticFade = new ToggleButton("Fade");
        this.btStaticFade.getStyleClass().add("ledButton");
//        this.btStaticFade.setOnAction(e -> this.setEffect(FoodComputerActions.STATIC_FADE));
        effectButtons.add(this.btStaticFade, 1, 0);

        this.btBlinking = new ToggleButton("Blink");
        this.btBlinking.getStyleClass().add("ledButton");
//        this.btBlinking.setOnAction(e -> this.setEffect(FoodComputerActions.BLINKING));
        effectButtons.add(this.btBlinking, 0, 1);

        this.btRunning = new ToggleButton("Run");
        this.btRunning.getStyleClass().add("ledButton");
//        this.btRunning.setOnAction(e -> this.setEffect(FoodComputerActions.RUNNING));
        effectButtons.add(this.btRunning, 1, 1);

        this.btStaticRainbow = new ToggleButton("Fixed rainbow");
        this.btStaticRainbow.getStyleClass().add("ledButton");
//        this.btStaticRainbow.setOnAction(e -> this.setEffect(FoodComputerActions.STATIC_RAINBOW));
        effectButtons.add(this.btStaticRainbow, 0, 2);

        this.btFadingRainbow = new ToggleButton("Fading rainbow");
        this.btFadingRainbow.getStyleClass().add("ledButton");
//        this.btFadingRainbow.setOnAction(e -> this.setEffect(FoodComputerActions.FADING_RAINBOW));
        effectButtons.add(this.btFadingRainbow, 1, 2);

        this.btWhite = new ToggleButton("White");
        this.btWhite.getStyleClass().add("ledButton");
//        this.btWhite.setOnAction(e -> this.setEffect(FoodComputerActions.ALL_WHITE));
        effectButtons.add(this.btWhite, 0, 3);

        this.btClear = new ToggleButton("Clear");
        this.btClear.getStyleClass().add("ledButton");
//        this.btClear.setOnAction(e -> this.setEffect(FoodComputerActions.ALL_OUT));
        effectButtons.add(this.btClear, 1, 3);

        effectButtons.add(new Label(""), 0, 4, 2, 1);

        Label lblSpeed = new Label("Speed");
        lblSpeed.getStyleClass().add("ledSpeed");
        GridPane.setHalignment(lblSpeed, HPos.CENTER);
        effectButtons.add(lblSpeed, 0, 5, 2, 1);

        this.slider = new Slider();
        this.slider.setShowTickLabels(true);
        this.slider.setShowTickMarks(true);
        this.slider.valueProperty().addListener((observable, oldValue, newValue) -> this.sendMessage());
        effectButtons.add(this.slider, 0, 6, 2, 1);

//        this.setEffect(FoodComputerActions.ALL_OUT);
    }

    /**
     * Handle the chosen effect from a button or a Mosquitto message to enable/disable the available UI elements and
     * highlight the button of the selected {@link FoodComputerAction}.
     *
     * @param foodComputerAction {@link FoodComputerAction}
     */
    private void setAction(FoodComputerAction foodComputerAction) {
        this.selectedFoodComputerAction = foodComputerAction;

//        this.colorSelector1.setDisable(!foodComputerActions.useColor1());
//        this.colorSelector2.setDisable(!foodComputerActions.useColor2());
//        this.slider.setDisable(!foodComputerActions.useSpeed());
//        this.slider.setMin(foodComputerActions.getMinimumSpeed());
//        this.slider.setMax(foodComputerActions.getMaximumSpeed());
//
//        this.btStatic.setSelected(foodComputerActions == FoodComputerActions.STATIC);
//        this.btStaticFade.setSelected(foodComputerActions == FoodComputerActions.STATIC_FADE);
//        this.btBlinking.setSelected(foodComputerActions == FoodComputerActions.BLINKING);
//        this.btRunning.setSelected(foodComputerActions == FoodComputerActions.RUNNING);
//        this.btStaticRainbow.setSelected(foodComputerActions == FoodComputerActions.STATIC_RAINBOW);
//        this.btFadingRainbow.setSelected(foodComputerActions == FoodComputerActions.FADING_RAINBOW);
//        this.btWhite.setSelected(foodComputerActions == FoodComputerActions.ALL_WHITE);
//        this.btClear.setSelected(foodComputerActions == FoodComputerActions.ALL_OUT);

        this.sendMessage();
    }


    /**
     * Send a message to Mosquitto if a new effect and/or different parameters are selected.
     */
    private void sendMessage() {
        if (this.slider == null) {
            // Not ready yet
            return;
        }

        if (this.blockSending) {
            // Avoid sending the same command to Mosquitto again to avoid infinite loops.
            return;
        }

        FoodComputerCommand foodComputerCommand = new FoodComputerCommand(
                this.selectedFoodComputerAction

//                this.selectedFoodComputerActions,
//                (int) this.slider.getValue(),
//                this.colorSelector1.getSelectedColor(),
//                this.colorSelector2.getSelectedColor()
        );

        System.out.println("Sending to Mosquitto: " + foodComputerCommand.toStringCommand());

        if (this.mqttClientConnection != null) {
            this.mqttClientConnection.sendMessage(foodComputerCommand, "foodComputerCommand");
        }
    }

    /**
     * {@link FoodComputerCommand} received from Mosquitto.
     * We block sending updates back to Mosquitto until the interface is updated fully to match the received command,
     * to avoid infinite loops.
     *
     * @param foodComputerCommand The {@link FoodComputerCommand}
     */
    @Override
    public void onQueueMessage(FoodComputerCommand foodComputerCommand) {
        this.blockSending = true;

        this.setAction(FoodComputerAction.fromId(foodComputerCommand.getId()));
//        this.slider.setValue(foodComputerCommand.getSpeed());
//        this.colorSelector1.setSelectedColor(foodComputerCommand.getColor1());
//        this.colorSelector2.setSelectedColor(foodComputerCommand.getColor2());

        this.blockSending = false;
    }
}
