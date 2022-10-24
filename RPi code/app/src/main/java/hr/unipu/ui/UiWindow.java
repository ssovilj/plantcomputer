package hr.unipu.ui;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import eu.hansolo.tilesfx.Tile;
import eu.hansolo.tilesfx.TileBuilder;
import eu.hansolo.tilesfx.tools.FlowGridPane;
import hr.unipu.client.MqttClientConnection;
import hr.unipu.event.EventListener;
import hr.unipu.event.EventManager;
import hr.unipu.plantcomputer.PlantComputerAction;
import hr.unipu.plantcomputer.PlantComputerCommand;
import hr.unipu.recipe.RecipeHandler;
import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ListView;
import javafx.scene.image.Image;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.Stop;
import javafx.scene.text.TextAlignment;
import javafx.stage.FileChooser;
import javafx.stage.Modality;
import javafx.stage.Stage;
import javafx.util.Duration;

import java.io.File;
import java.util.*;
import java.util.List;

/**
 * A new UI for Food Computer.
 */
public class UiWindow extends FlowGridPane implements EventListener {
    private static final    double TILE_WIDTH  = 150;
    private static final    double TILE_HEIGHT = 150;
    private static final    int NO_OF_COLS = 6;
    private static final    int NO_OF_ROWS = 5;
    private static final    int SPACING = 5;

    private static MqttClientConnection mqttClientConnection = null;
    private final EventManager eventManager;
    private static boolean blockSending = false;

    public static PlantComputerAction selectedPlantComputerAction = PlantComputerAction.UNDEFINED;
    public static List<PlantComputerCommand> listActuatorCommands = new ArrayList<>();
    public static List<PlantComputerCommand> listReadingCommands = new ArrayList<>();
    public static List<PlantComputerCommand> listStateReadings = new ArrayList<>();

    private static final Random RND = new Random();

    private final VBox home;
    private final VBox sensorsVBox1;
    private final VBox sensorsVbox2;
    private final VBox actuatorsVbox1;
    private final VBox actuatorsVbox2;
    private final VBox logs;
    private final VBox recipes;

    private final Tile imageTile;

    private final Tile btGrowLight;
    private final Tile btHumidifier;
    private final Tile btCoolingFan;
    private final Tile btChamberFan;
    private final Tile btHeater;
    private final Tile btWaterCirculationPump;

    private final Tile temperatureTile;
    private final Tile recipesBoardTile;


    // Countdown timer
    public static Tile countdownTile;
    public final HBox countdownHbox;
    public final VBox countdownVbox1;
    public final VBox countdownVbox2;
    private static final int            SECONDS_PER_DAY    = 86_400;
    private static final int            SECONDS_PER_HOUR   = 3600;
    private static final int            SECONDS_PER_MINUTE = 60;
    private static Tile           days;
    private static Tile           hours;
    private static Tile           minutes;
    private static Tile           seconds;
    private static Duration duration;


    private static boolean recipeSelected = false;
    public static TimerTask task;

    private final Tile sparkLineSensor1;
    private final Tile sparkLineSensor2;
    private final Tile sparkLineSensor3;
    private final Tile sparkLineSensor4;
    private final Tile sparkLineSensor5;
    private final Tile sparkLineSensor6;

    private final Tile barGaugeSensor1;
    private final Tile barGaugeSensor2;
    private final Tile barGaugeSensor3;
    private final Tile barGaugeSensor4;
    private final Tile barGaugeSensor5;
    private final Tile barGaugeSensor6;

    private static Tile textTile = null;
    private final Tile cameraTile;
    private final Tile customTile;
    private static LinkedList<String> logText = new LinkedList<>();

    private static AnimationTimer  timer1;
    private static long lastTimerCall1;
    public static AnimationTimer  timer2;
    private static long lastTimerCall2;


    public static DoubleProperty value1;    // SATM 1
    public static DoubleProperty value2;    // SAHU 1
    public static DoubleProperty value3;    // SWEC 1
    public static DoubleProperty value4;    // SWPH 1
    public static DoubleProperty value5;    // SWTM 1
    public static DoubleProperty value6;    // SLIN 1

    double target1 = 27.0;      // [°C]
    double target2 = 60.0;      // [%]
    double target3 = 1.5;       // [mS/cm]
    double target4 = 6.4;       // []
    double target5 = 18.33;     // [°C]
    double target6 = 15000.0;   // [lux]


    // Instantiations of all tile nodes.
    public UiWindow(EventManager eventManager, MqttClientConnection mqttClientConnection) {
        super(NO_OF_COLS, NO_OF_ROWS);
        super.setHgap(SPACING);
        super.setVgap(SPACING);
        super.setAlignment(Pos.TOP_LEFT);
        super.setCenterShape(true);
        super.setPadding(new Insets(5));
        //super.setPrefSize(1280, 720);    // original: 800x600, best: 1280x720 (16:9)
        super.setBackground(new Background(new BackgroundFill(Color.web("#101214"), CornerRadii.EMPTY, Insets.EMPTY)));
        //UiWindow.init(Duration.hours(0));

        UiWindow.mqttClientConnection = mqttClientConnection;
        this.eventManager = eventManager;
        eventManager.addListener(this);     // Add listener to UI Window.


        value1 = new SimpleDoubleProperty(0);
        value2 = new SimpleDoubleProperty(0);
        value3 = new SimpleDoubleProperty(0);
        value4 = new SimpleDoubleProperty(0);
        value5 = new SimpleDoubleProperty(0);
        value6 = new SimpleDoubleProperty(0);


        /**
         * ---HOME group---
         */

        // Home group pane.
        this.home = new VBox();

        // Image tile: UNIPU/FIPU.
        imageTile = TileBuilder.create()
                .skinType(Tile.SkinType.IMAGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT * 1)
                .title("Food Computer in Java/Kotlin")
                .titleAlignment(TextAlignment.CENTER)
                .image(new Image(UiWindow.class.getResourceAsStream("/unipu-lat.png")))
                .imageMask(Tile.ImageMask.NONE)
                .text("ver.1.0")
                .textAlignment(TextAlignment.CENTER)
                .build();

        temperatureTile = TileBuilder.create().skinType(Tile.SkinType.FIRE_SMOKE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Air Temperature".toUpperCase())
                //.text("Air temperature")
                .unit("\u00b0C")
                .threshold(target1 * 2) // triggers the fire and smoke effect
                .decimals(1)
                .animated(true)
                .build();
        temperatureTile.valueProperty().bind(value1);


        ObservableList recipesList = FXCollections.observableArrayList();
        ListView recipesListView = new ListView(recipesList);
        recipesListView.setStyle("-fx-control-inner-background: black;");
        Button addRecipeButton = new Button("Add recipe".toUpperCase());
        addRecipeButton.setOnAction(e -> {
            final Stage newWindow = new Stage();
            newWindow.initModality(Modality.NONE);
            FileChooser fileChooser = new FileChooser();
            fileChooser.setTitle("Open JSON Recipe File");
            fileChooser.setInitialDirectory(new File(System.getProperty("user.dir"),"/build/resources/main/recipes"));
            fileChooser.getExtensionFilters().addAll(
                    new FileChooser.ExtensionFilter("JSON recipes", "*.json")
            );
            File file = fileChooser.showOpenDialog(newWindow);
            if (file != null) {
                recipesList.add(file.getName());
            }
        });
        recipesListView.getSelectionModel().selectedItemProperty().addListener(new ChangeListener() {
            @Override
            public void changed(ObservableValue observable, Object oldValue, Object newValue) {
                recipeSelected = newValue != null;

                task = new TimerTask() {
                    @Override
                    public void run() {
                        System.out.println("Selected recipe: " + newValue);

                        // Stop task if unselected.
                        if (!recipeSelected) {
                            task.cancel();
                            task = null;
                            System.out.println("Task canceled.");
                            countdownTile.setVisible(false);
                            return;
                        }

                        // Set Recipe counter before, since is in while loop after.
                        Platform.runLater(() -> {
                            // Update UI thread from here (if needed).
                        });

                        // Run recipe.
                        System.out.println("Running recipe handler...");
                        RecipeHandler.run_recipe((String) newValue);

                    }
                };

                new Thread(task).start();
            }
        });
        this.recipes = new VBox();
        this.recipes.getChildren().addAll(recipesListView, addRecipeButton);

        recipesBoardTile = TileBuilder.create()
                .skinType(Tile.SkinType.CUSTOM)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Recipes".toUpperCase())
                .text("Click recipe to run, CTRL+click to stop")
                .graphic(recipes)
                .roundedCorners(true)
                .build();

//        countdownTile = TileBuilder.create()
//                .skinType(Tile.SkinType.COUNTDOWN_TIMER)
//                .prefSize(TILE_WIDTH, TILE_HEIGHT)
//                .title("Recipe Countdown Timer".toUpperCase())
//                .description("Description")
//                //.text("Text")
//                .barColor(Bright.ORANGE_RED)
//                .timePeriod(Duration.ofSeconds(0))
//                .hourTickMarksVisible(true)
//                .onAlarm(e -> System.out.println("Recipe finished."))
//                .build();

        countdownHbox = new HBox();
        countdownVbox1 = new VBox();
        countdownVbox2 = new VBox();
        days     = createTile("DAYS", "0");
        hours    = createTile("HOURS", "0");
        minutes  = createTile("MINUTES", "0");
        seconds  = createTile("SECONDS", "0");
        countdownVbox1.getChildren().addAll(days, hours);
        countdownVbox2.getChildren().addAll(minutes, seconds);
        countdownHbox.getChildren().addAll(countdownVbox1, countdownVbox2);
        countdownHbox.setPadding(new Insets(10));

        countdownTile = TileBuilder.create()
                .skinType(Tile.SkinType.CUSTOM)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Recipe Countdown Timer".toUpperCase())
                .graphic(countdownHbox)
                .roundedCorners(true)
                .build();
        countdownTile.setVisible(false);
        //timer2.start();

        this.home.getChildren().addAll(imageTile,
                                        temperatureTile,
                                        countdownTile,
                                        recipesBoardTile
                                        );
        this.home.setSpacing(SPACING);


        /**
         * ---SENSORS group---
         */

        // Sensors group pane.
        this.sensorsVBox1 = new VBox();

        // Air Temperature [°C], target 27°C.
        sparkLineSensor1= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Air Temperature".toUpperCase())
                .unit("\u00b0C")
                .valueVisible(true)
                .decimals(1)
                .gradientStops(new Stop(0.00000, Color.TRANSPARENT),
                        new Stop(0.00001, Color.web("#3552a0")),
                        new Stop(0.09090, Color.web("#456acf")),
                        new Stop(0.27272, Color.web("#45a1cf")),
                        new Stop(0.36363, Color.web("#30c8c9")),
                        new Stop(0.45454, Color.web("#30c9af")),
                        new Stop(0.50909, Color.web("#56d483")),
                        new Stop(0.72727, Color.web("#9adb49")),
                        new Stop(0.81818, Color.web("#efd750")),
                        new Stop(0.90909, Color.web("#ef9850")),
                        new Stop(1.00000, Color.web("#ef6050")))
                .minValue(0)
                .maxValue(target1 * 2)
                .strokeWithGradient(true)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor1.valueProperty().bind(value1);

        // Air Humidity [%], target 60%.
        sparkLineSensor2= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Air Humidity".toUpperCase())
                .unit("%")
                .decimals(1)
                .gradientStops(new Stop(0.0, Tile.RED),     // Too dry.
                        new Stop(0.4, Tile.GREEN),
                        new Stop(1.0, Tile.BLUE))           // Too wet.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target2 * 2)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor2.valueProperty().bind(value2);

        // Water Electrical Conductivity [mS/cm], target 1.5mS/cm.
        sparkLineSensor3= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Water EC".toUpperCase())
                .unit("mS/cm")
                .decimals(1)
                .gradientStops(new Stop(0, Tile.GRAY), // Too low nutrient solution.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))      // Too much nutrient solution.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target3 * 2)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor3.valueProperty().bind(value3);

        // Water pH [], target 6.4;
        sparkLineSensor4= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Water pH".toUpperCase())
                .unit(" ")
                .decimals(1)
                .gradientStops(new Stop(0, Tile.RED),   // Too acid.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.BLUE))      // Too alkaline.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target4 * 2)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor4.valueProperty().bind(value4);

        // Water temperature [C]
        sparkLineSensor5= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Water Temperature".toUpperCase())
                .unit("\u00b0C")
                .decimals(1)
                .gradientStops(new Stop(0, Tile.BLUE),  // Water temperature too low.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))        // Water temperature too high.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target5 * 2)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor5.valueProperty().bind(value5);

        // Light Intensity [lux]
        sparkLineSensor6= TileBuilder.create()
                .skinType(Tile.SkinType.SPARK_LINE)
                .prefSize(TILE_WIDTH * 3, TILE_HEIGHT)
                .title("Light Intensity".toUpperCase())
                .unit("lux")
                .decimals(0)
                .gradientStops(new Stop(0, Tile.BLUE),  // Light intensity too low.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))        // Light intensity too high.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target6 * 2)
                //.smoothing(true)
                .fixedYScale(true)
                .build();
        sparkLineSensor6.valueProperty().bind(value6);

        this.sensorsVBox1.getChildren().addAll(sparkLineSensor1,
                sparkLineSensor2,
                sparkLineSensor3,
                sparkLineSensor4,
                sparkLineSensor5,
                sparkLineSensor6);
        this.sensorsVBox1.setSpacing(SPACING);


        // Sensors group pane.
        this.sensorsVbox2 = new VBox();

        barGaugeSensor1 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Air Temperature".toUpperCase())
                .text("\"SATM 1\"")
                .unit("\u00b0C")
                .textVisible(true)
                .value(0)
                .valueVisible(true)
                .gradientStops(new Stop(0.00000, Color.TRANSPARENT),
                        new Stop(0.00001, Color.web("#3552a0")),
                        new Stop(0.09090, Color.web("#456acf")),
                        new Stop(0.27272, Color.web("#45a1cf")),
                        new Stop(0.36363, Color.web("#30c8c9")),
                        new Stop(0.45454, Color.web("#30c9af")),
                        new Stop(0.50909, Color.web("#56d483")),
                        new Stop(0.72727, Color.web("#9adb49")),
                        new Stop(0.81818, Color.web("#efd750")),
                        new Stop(0.90909, Color.web("#ef9850")),
                        new Stop(1.00000, Color.web("#ef6050")))
                .strokeWithGradient(true)
                .threshold(target1)
                .thresholdVisible(true)
                .minValue(0)
                .maxValue(target1 * 2)
                .decimals(1)
                .tickLabelDecimals(1)
                .animated(true)
                .build();
        barGaugeSensor1.valueProperty().bind(value1);

        barGaugeSensor2 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Air Humidity".toUpperCase())
                .text("\"SAHU 1\"")
                .unit("%")
                .textVisible(true)
                .value(0)
                .gradientStops(new Stop(0.0, Tile.RED),   // Too dry.
                        new Stop(0.4, Tile.GREEN),
                        new Stop(1.0, Tile.BLUE))         // Too wet.
                .strokeWithGradient(true)
                .minValue(0)        // 0%
                .maxValue(100)      // 100%
                .threshold(target2)
                .thresholdVisible(true)
                .decimals(1)
                .tickLabelDecimals(1)
                .animated(true)
                .build();
        barGaugeSensor2.valueProperty().bind(value2);

        barGaugeSensor3 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Water EC".toUpperCase())
                .text("\"SWEC 1\"")
                .unit("mS/cm")
                .textVisible(true)
                .value(0)
                .gradientStops(new Stop(0, Tile.GRAY), // Too low nutrient solution.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))       // Too much nutrient solution.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target3 * 2)
                .threshold(target3)
                .thresholdVisible(true)
                .decimals(1)
                .tickLabelDecimals(1)
                .animated(true)
                .build();
        barGaugeSensor3.valueProperty().bind(value3);

        barGaugeSensor4 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Water pH".toUpperCase())
                .text("\"SWPH 1\"")
                .unit(" ")
                .textVisible(true)
                .value(0)
                .gradientStops(new Stop(0, Tile.RED),   // Too acid.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.BLUE))           // Too alkaline.
                .strokeWithGradient(true)
                .minValue(0)        // 0
                .maxValue(10)       // 10
                .threshold(target4)
                .thresholdVisible(true)
                .decimals(1)
                .tickLabelDecimals(1)
                .animated(true)
                .build();
        barGaugeSensor4.valueProperty().bind(value4);

        barGaugeSensor5 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Water Temperature".toUpperCase())
                .text("\"SWTM 1\"")
                .unit("\u00b0C")
                .textVisible(true)
                .value(0)
                .gradientStops(new Stop(0, Tile.BLUE),  // Water temperature too low.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))        // Water temperature too high.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target5 * 2)
                .threshold(target5)
                .thresholdVisible(true)
                .decimals(1)
                .tickLabelDecimals(1)
                .animated(true)
                .build();
        barGaugeSensor5.valueProperty().bind(value5);

        barGaugeSensor6 = TileBuilder.create()
                .skinType(Tile.SkinType.BAR_GAUGE)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Light Intensity".toUpperCase())
                .text("\"SLIN 1\"")
                .unit("lux")
                .textVisible(true)
                .value(0)
                .gradientStops(new Stop(0, Tile.BLUE),  // Light intensity too low.
                        new Stop(0.5, Tile.GREEN),
                        new Stop(1.0, Tile.RED))        // Light intensity too high.
                .strokeWithGradient(true)
                .minValue(0)
                .maxValue(target6 * 2)
                .threshold(target6)
                .thresholdVisible(true)
                .decimals(0)
                .tickLabelDecimals(0)
                .animated(true)
                .build();
        barGaugeSensor6.valueProperty().bind(value6);

        this.sensorsVbox2.getChildren().addAll(barGaugeSensor1,
                barGaugeSensor2,
                barGaugeSensor3,
                barGaugeSensor4,
                barGaugeSensor5,
                barGaugeSensor6);
        this.sensorsVbox2.setSpacing(SPACING);


        /**
         * ---ACTUATORS group---
         */

        // Actuators group pane.
        this.actuatorsVbox1 = new VBox();
        this.actuatorsVbox2 = new VBox();

        btGrowLight = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Grow Lights".toUpperCase())
                .text("\"APLN 1\"")
                //.description("Description text")
                .build();
        btGrowLight.setOnSwitchPressed(e -> {
            //System.out.println("Switch pressed.");
            if (this.btGrowLight.isActive()) {
                //System.out.println("Switch is active.");
                this.setAction(PlantComputerAction.LIGHT_TURN_ON);
            } else {
                //System.out.println("Switch is NOT active.");
                this.setAction(PlantComputerAction.LIGHT_TURN_OFF);
            }
        });


        btHumidifier = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Humidifier".toUpperCase())
                .text("\"AAHU 1\"")
                //.description("Description text")
                .build();
        btHumidifier.setOnSwitchPressed(e -> {
            if (this.btHumidifier.isActive()) {
                this.setAction(PlantComputerAction.HUMIDIFIER_TURN_ON);
            } else {
                this.setAction(PlantComputerAction.HUMIDIFIER_TURN_OFF);
            }
        });


        btCoolingFan = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Cooling Fan".toUpperCase())
                .text("\"AAVE 1\"")
                //.description("Description text")
                .build();
        btCoolingFan.setOnSwitchPressed(e -> {
            if (this.btCoolingFan.isActive()) {
                this.setAction(PlantComputerAction.CHILLER_FAN_TURN_ON);
            } else {
                this.setAction(PlantComputerAction.CHILLER_FAN_TURN_OFF);
            }
        });


        btChamberFan = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Cahmber Fan".toUpperCase())
                .text("\"AACR 1\"")
                //.description("Description text")
                .build();
        btChamberFan.setOnSwitchPressed(e -> {
            if (this.btChamberFan.isActive()) {
                this.setAction(PlantComputerAction.CHAMBER_FAN_TURN_ON);
            } else {
                this.setAction(PlantComputerAction.CHAMBER_FAN_TURN_OFF);
            }
        });


        btHeater = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Heater Core".toUpperCase())
                .text("\"AAHE 1\"")
                //.description("Description text")
                .build();
        btHeater.setOnSwitchPressed(e -> {
            if (this.btHeater.isActive()) {
                this.setAction(PlantComputerAction.HEATER_TURN_ON);
            } else {
                this.setAction(PlantComputerAction.HEATER_TURN_OFF);
            }
        });


        btWaterCirculationPump = TileBuilder.create()
                .skinType(Tile.SkinType.SWITCH)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Water Circulation Pump".toUpperCase())
                .text("\"AAWP 1\"")
                //.description("Description text")
                .build();
        btWaterCirculationPump.setOnSwitchPressed(e -> {
            if (this.btWaterCirculationPump.isActive()) {
                this.setAction(PlantComputerAction.WATER_PUMP_TURN_ON);
            } else {
                this.setAction(PlantComputerAction.WATER_PUMP_TURN_OFF);
            }
        });


        this.actuatorsVbox1.getChildren().addAll(btGrowLight, btHumidifier,
                                            btCoolingFan, btChamberFan);
        this.actuatorsVbox2.getChildren().addAll(btHeater,
                                            btWaterCirculationPump);
        this.actuatorsVbox1.setSpacing(SPACING);
        this.actuatorsVbox2.setSpacing(SPACING);

        /**
         * ---LOGS---
         */
        this.logs = new VBox();

        cameraTile = TileBuilder.create()
                .skinType(Tile.SkinType.IMAGE)
                .prefSize(TILE_WIDTH * 2, TILE_HEIGHT * 2 + SPACING * 1)
                .title("CAMERA")
                .image(new Image(UiWindow.class.getResourceAsStream("/camera1.jpg")))
                .imageMask(Tile.ImageMask.NONE)
                //.text("Whatever text")
                .textSize(Tile.TextSize.SMALLER)
                .textVisible(false)
                .textAlignment(TextAlignment.CENTER)
                .build();

        //TODO Real camera.

        textTile = TileBuilder.create()
                .skinType(Tile.SkinType.TEXT)
                .prefSize(TILE_WIDTH*2, TILE_HEIGHT * 2 + SPACING * 1)
                .title("Logs".toUpperCase())
                .description(logText.toString())
                .descriptionAlignment(Pos.TOP_RIGHT)
                //.text("Whatever text")
                .textSize(Tile.TextSize.SMALLER)
                .textVisible(false)
                .build();

        Button logButton = new Button("LOGS");
        logButton.setOnAction(e -> {
            final Stage newWindow = new Stage();
            newWindow.initModality(Modality.NONE);
            LogsPanel logsPanel = new LogsPanel();
            eventManager.addListener(logsPanel);
            Scene logsScene = new Scene(logsPanel, 800, 600);
            newWindow.setScene(logsScene);
            newWindow.show();
        });
        customTile = TileBuilder.create()
                .skinType(Tile.SkinType.CUSTOM)
                .prefSize(TILE_WIDTH, TILE_HEIGHT)
                .title("Log details".toUpperCase())
                //.text("Whatever text")
                .graphic(logButton)
                .roundedCorners(false)
                .build();

        this.logs.getChildren().addAll(cameraTile, textTile, customTile);
        this.logs.setSpacing(SPACING);

        super.getChildren().addAll(home,
                sensorsVBox1, sensorsVbox2,
                actuatorsVbox1, actuatorsVbox2, logs);


        lastTimerCall1 = System.nanoTime();
        timer1 = new AnimationTimer() {
            @Override public void handle(long now) {
                if (now > lastTimerCall1 + 10_000_000_000L) {     // Update every 10.0 seconds.

                    // Sending command for reading sensors & actuators to MQTT broker.
                    listReadingCommands.clear();
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.TEMPERATURE_AIR_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.HUMIDITY_AIR_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.CONDUCTIVITY_WATER_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.PH_WATER_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.TEMPERATURE_WATER_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.LIGHT_INTENSITY_STATE_READ));

                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.LIGHT_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.HUMIDIFIER_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.COOLING_FAN_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.CHAMBER_FAN_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.HEATER_TURN_STATE_READ));
                    listReadingCommands.add(new PlantComputerCommand(PlantComputerAction.WATER_PUMP_STATE_READ));

                    UiWindow.sendMessageForReadingStates();


                    //sparkLineSensor1.setValue(RND.nextDouble() * sparkLineSensor1.getRange() * 1.0 + sparkLineSensor1.getMinValue());
                    //value1.set(RND.nextDouble() * sparkLineSensor1.getRange() * 1.0 + sparkLineSensor1.getMinValue());
                    Double rndValue1 = RND.nextDouble() * sparkLineSensor1.getRange() * 1.0 + sparkLineSensor1.getMinValue();
                    rndValue1 = RND.nextDouble() * sparkLineSensor1.getRange() * 0.1 + target1;
                    //sparkLineSensor1.setValue(20);

                    //sparkLineSensor2.setValue(RND.nextDouble() * sparkLineSensor2.getRange() * 1.0 + sparkLineSensor2.getMinValue());
                    //value2.set(RND.nextDouble() * sparkLineSensor2.getRange() * 1.0 + sparkLineSensor2.getMinValue());
                    Double rndValue2 = RND.nextDouble() * sparkLineSensor2.getRange() * 1.0 + sparkLineSensor2.getMinValue();
                    rndValue2 = RND.nextDouble() * sparkLineSensor2.getRange() * 0.1 + target2;
                    //sparkLineSensor2.setValue(20);

                    //sparkLineSensor3.setValue(RND.nextDouble() * sparkLineSensor3.getRange() * 1.0 + sparkLineSensor3.getMinValue());
                    //value3.set(RND.nextDouble() * sparkLineSensor3.getRange() * 1.0 + sparkLineSensor3.getMinValue());
                    Double rndValue3 = RND.nextDouble() * sparkLineSensor3.getRange() * 1.0 + sparkLineSensor3.getMinValue();
                    rndValue3 = RND.nextDouble() * sparkLineSensor3.getRange() * 0.1 + target3;
                    //sparkLineSensor3.setValue(20);

                    //sparkLineSensor4.setValue(RND.nextDouble() * sparkLineSensor4.getRange() * 1.0 + sparkLineSensor4.getMinValue());
                    //value4.set(RND.nextDouble() * sparkLineSensor4.getRange() * 1.0 + sparkLineSensor4.getMinValue());
                    Double rndValue4 = RND.nextDouble() * sparkLineSensor4.getRange() * 1.0 + sparkLineSensor4.getMinValue();
                    rndValue4 = RND.nextDouble() * sparkLineSensor4.getRange() * 0.1 + target4;
                    //sparkLineSensor4.setValue(20);

                    //sparkLineSensor5.setValue(RND.nextDouble() * sparkLineSensor5.getRange() * 1.0 + sparkLineSensor5.getMinValue());
                    //value5.set(RND.nextDouble() * sparkLineSensor5.getRange() * 1.0 + sparkLineSensor5.getMinValue());
                    Double rndValue5 = RND.nextDouble() * sparkLineSensor5.getRange() * 1.0 + sparkLineSensor5.getMinValue();
                    rndValue5 = RND.nextDouble() * sparkLineSensor5.getRange() * 0.1 + target5;
                    //sparkLineSensor5.setValue(20);

                    //sparkLineSensor6.setValue(RND.nextDouble() * sparkLineSensor6.getRange() * 1.0 + sparkLineSensor6.getMinValue());
                    //value6.set(RND.nextDouble() * sparkLineSensor6.getRange() * 1.0 + sparkLineSensor6.getMinValue());
                    Double rndValue6 = RND.nextDouble() * sparkLineSensor6.getRange() * 1.0 + sparkLineSensor6.getMinValue();
                    rndValue6 = RND.nextDouble() * sparkLineSensor6.getRange() * 0.1 + target6;
                    //sparkLineSensor6.setValue(20);

                    /**
                     * TESTING: sensor readings as MQTT message.
                     */
                    // Sending command for reading sensors to MQTT broker.
                    listStateReadings.clear();
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.TEMPERATURE_AIR_STATE.setActionValue(rndValue1.toString())));
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.HUMIDITY_AIR_STATE.setActionValue(rndValue2.toString())));
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.CONDUCTIVITY_WATER_STATE.setActionValue(rndValue3.toString())));
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.PH_WATER_STATE.setActionValue(rndValue4.toString())));
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.TEMPERATURE_WATER_STATE.setActionValue(rndValue5.toString())));
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.LIGHT_INTENSITY_STATE.setActionValue(rndValue6.toString())));

                    String isBtGrowLightOn; if (btGrowLight.isActive())  isBtGrowLightOn="on"; else isBtGrowLightOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.LIGHT_STATE.setActionValue(isBtGrowLightOn)));
                    String isBtHumidifierOn; if (btHumidifier.isActive())  isBtHumidifierOn="on"; else isBtHumidifierOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.HUMIDIFIER_STATE.setActionValue(isBtHumidifierOn)));
                    String isBtCoolingFanOn; if (btCoolingFan.isActive())  isBtCoolingFanOn="on"; else isBtCoolingFanOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.COOLING_FAN_STATE.setActionValue(isBtCoolingFanOn)));
                    String isBtChamberFanOn; if (btChamberFan.isActive())  isBtChamberFanOn="on"; else isBtChamberFanOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.CHAMBER_FAN_STATE.setActionValue(isBtChamberFanOn)));
                    String isBtHeaterOn; if (btHeater.isActive())  isBtHeaterOn="on"; else isBtHeaterOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.HEATER_TURN_STATE.setActionValue(isBtHeaterOn)));
                    String isBtWaterCirculationPumpOn; if (btWaterCirculationPump.isActive())  isBtWaterCirculationPumpOn="on"; else isBtWaterCirculationPumpOn="off";
                    listStateReadings.add(new PlantComputerCommand(PlantComputerAction.WATER_PUMP_STATE.setActionValue(isBtWaterCirculationPumpOn)));

                    UiWindow.sendMessageStateReadings();

                    // Update UI after state readings received. (REDUNDANT SINCE BINDING)
//                    temperatureTile.setValue(sparkLineSensor1.getValue());
//                    barGaugeSensor1.setValue(sparkLineSensor1.getValue());
//                    barGaugeSensor2.setValue(sparkLineSensor2.getValue());
//                    barGaugeSensor3.setValue(sparkLineSensor3.getValue());
//                    barGaugeSensor4.setValue(sparkLineSensor4.getValue());
//                    barGaugeSensor5.setValue(sparkLineSensor5.getValue());

                    lastTimerCall1 = now;
                }
            }
        };
        timer1.start();
    }

    /**
     * Handle the chosen effect from a button or a Mosquitto message to enable/disable the available UI elements and
     * highlight the button of the selected {@link PlantComputerAction}.
     *
     * @param plantComputerAction {@link PlantComputerAction}
     */
    private void setAction(PlantComputerAction plantComputerAction) {
        // Single actuator command.
        UiWindow.selectedPlantComputerAction = plantComputerAction;

        // Creating list of actuator commands.
        UiWindow.listActuatorCommands.clear();
        if (btGrowLight.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.LIGHT_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.LIGHT_TURN_OFF));
        if (btHumidifier.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.HUMIDIFIER_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.HUMIDIFIER_TURN_OFF));
        if (btCoolingFan.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.CHILLER_FAN_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.CHILLER_FAN_TURN_OFF));
        if (btChamberFan.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.CHAMBER_FAN_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.CHAMBER_FAN_TURN_OFF));
        if (btHeater.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.HEATER_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.HEATER_TURN_OFF));
        if (btWaterCirculationPump.isActive())
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.WATER_PUMP_TURN_ON));
        else
            listActuatorCommands.add(new PlantComputerCommand(PlantComputerAction.WATER_PUMP_TURN_OFF));

        UiWindow.sendMessageForActuators();
    }

    /**
     * Send a message to Mosquitto if a new action and/or different parameters are selected.
     */
    public static void sendMessageForActuators() {

        if (UiWindow.blockSending) {
            // Avoid sending the same command to Mosquitto again to avoid infinite loops.
            return;
        }

        // Create single actuator command.
        PlantComputerCommand plantComputerCommand = new PlantComputerCommand(
                UiWindow.selectedPlantComputerAction
        );

        //System.out.println("Sending String command to Mosquitto: " + plantComputerCommand.toStringCommand());
        //System.out.println("Sending JSON command to Mosquitto: " + plantComputerCommand.toJsonCommand());

        /**
         * Sending message command for actuators to MQTT broker.
         */
        final boolean mqttClientConnectionExist = mqttClientConnection != null;
        if (mqttClientConnectionExist) {
            final boolean isMqttClientConnected = mqttClientConnection.getMqttClient().isConnected();
            if (isMqttClientConnected) {
                //this.mqttClientConnection.sendMessage(plantComputerCommand, "plantComputerCommand");  // Sending single selected command.
                final ObjectMapper mapper = new ObjectMapper();
                try {
                    // Sending list of commands for actuators.
                    //System.out.println("Sending commands for actuators:\n" + mapper.writeValueAsString(listActuatorCommands));
                    mqttClientConnection.sendMessage(mapper.writeValueAsString(listActuatorCommands), "plantComputerCommand");
                } catch (JsonProcessingException e) {
                    e.printStackTrace();
                }
            }
        }

        /**
         * Adding short log to UI.
         */
        logText.add(plantComputerCommand.toStringCommand());
        if (logText.size() > 5) logText.remove();      // FIFO, remove the oldest.
        LinkedList logTextReversed = (LinkedList) logText.clone();
        Collections.reverse(logTextReversed);
        textTile.setDescription(logTextReversed.toString()
                .replace("[","")
                .replace("]","")
                .replace(",",""));
    }


    /**
     * Send a message to Mosquitto for reading sensors' and actuators' states.
     */
    private static void sendMessageForReadingStates() {

        if (UiWindow.blockSending) {
            // Avoid sending the same command to Mosquitto again to avoid infinite loops.
            return;
        }

        //System.out.println("Sending String command to Mosquitto: " + plantComputerCommand.toStringCommand());
        //System.out.println("Sending JSON command to Mosquitto: " + plantComputerCommand.toJsonCommand());

        // Sending message command for actuators to MQTT broker.
        final boolean mqttClientConnectionExist = mqttClientConnection != null;
        if (mqttClientConnectionExist) {
            final boolean isMqttClientConnected = mqttClientConnection.getMqttClient().isConnected();
            if (isMqttClientConnected) {
                final ObjectMapper mapper = new ObjectMapper();
                try {
                    // Sending list of commands for actuators.
                    //System.out.println("Sending commands for sensors:\n" + mapper.writeValueAsString(listSensorCommands));
                    mqttClientConnection.sendMessage( mapper.writeValueAsString(listReadingCommands), "plantComputerCommand");
                } catch (JsonProcessingException e) {
                    e.printStackTrace();
                }
            }
        }

    }


    /**
     * Send a message to Mosquitto for reading sensors and actuators state.
     */
    private static void sendMessageStateReadings() {

        if (UiWindow.blockSending) {
            // Avoid sending the same command to Mosquitto again to avoid infinite loops.
            return;
        }

        //System.out.println("Sending String command to Mosquitto: " + plantComputerCommand.toStringCommand());
        //System.out.println("Sending JSON command to Mosquitto: " + plantComputerCommand.toJsonCommand());

        // Sending message "test" readings for sensors and actuators to MQTT broker.
        final boolean mqttClientConnectionExist = UiWindow.mqttClientConnection != null;
        if (mqttClientConnectionExist) {
            final boolean isMqttClientConnected = UiWindow.mqttClientConnection.getMqttClient().isConnected();
            if (isMqttClientConnected) {
                final ObjectMapper mapper = new ObjectMapper();
                try {
                    // Sending list of readings of sensors.
                    //System.out.println("Sending readings from sensors:\n" + mapper.writeValueAsString(listSensorReadings));
                    UiWindow.mqttClientConnection.sendMessage( mapper.writeValueAsString(listStateReadings), "plantComputerState");
                } catch (JsonProcessingException e) {
                    e.printStackTrace();
                }
            }
        }

    }


    /**
     * {@link PlantComputerCommand} received from Mosquitto.
     *
     *
     * @param plantComputerCommand The {@link PlantComputerCommand}
     */
    @Override
    public void onQueueMessage(PlantComputerCommand plantComputerCommand) {
        UiWindow.blockSending = true;

        System.out.println("Command received from Mosquitto: " + plantComputerCommand.toStringCommand());

        // Sensor readings + update values in UI:
        if (plantComputerCommand.getId().equals("500") && plantComputerCommand.getActionName().equals("SATM 1")) {
            value1.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        } else if (plantComputerCommand.getId().equals("510") && plantComputerCommand.getActionName().equals("SAHU 1")) {
            value2.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        } else if (plantComputerCommand.getId().equals("520") && plantComputerCommand.getActionName().equals("SWEC 1")) {
            value3.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        } else if (plantComputerCommand.getId().equals("530") && plantComputerCommand.getActionName().equals("SWPH 1")) {
            value4.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        } else if (plantComputerCommand.getId().equals("540") && plantComputerCommand.getActionName().equals("SWTM 1")) {
            value5.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        } else if (plantComputerCommand.getId().equals("550") && plantComputerCommand.getActionName().equals("SLIN 1")) {
            value6.set(Double.parseDouble(plantComputerCommand.getActionValue()));
        }

        // Redundant
        //this.setAction(FoodComputerAction.fromId(plantComputerCommand.getId()));

        UiWindow.blockSending = false;

    }

    public static boolean isRecipeSelected() {
        return recipeSelected;
    }

    public static void init(Duration countDownTime) {

        duration = countDownTime;

        lastTimerCall2 = System.nanoTime();
        if (timer2 != null) timer2=null;  // Reset timer.
        timer2 = new AnimationTimer() {
            @Override public void handle(final long now) {
                if (now > lastTimerCall2 + 1_000_000_000l) {
                    duration = duration.subtract(Duration.seconds(1));

                    int remainingSeconds = (int) duration.toSeconds();
                    int d = remainingSeconds / SECONDS_PER_DAY;
                    int h = (remainingSeconds % SECONDS_PER_DAY) / SECONDS_PER_HOUR;
                    int m = ((remainingSeconds % SECONDS_PER_DAY) % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
                    int s = (((remainingSeconds % SECONDS_PER_DAY) % SECONDS_PER_HOUR) % SECONDS_PER_MINUTE);

                    if (d == 0 && h == 0 && m == 0 && s == 0) { timer2.stop(); }

                    days.setDescription(Integer.toString(d));
                    hours.setDescription(Integer.toString(h));
                    minutes.setDescription(String.format("%02d", m));
                    seconds.setDescription(String.format("%02d", s));

                    lastTimerCall2 = now;
                }
            }
        };
    }

    private static Tile createTile(final String TITLE, final String TEXT) {
        return TileBuilder.create().skinType(Tile.SkinType.CHARACTER)
                .prefSize(TILE_WIDTH/5, TILE_HEIGHT/5)
                .title(TITLE)
                .textSize(Tile.TextSize.BIGGER)
                .titleAlignment(TextAlignment.CENTER)
                .description(TEXT)
                .build();
    }

}
