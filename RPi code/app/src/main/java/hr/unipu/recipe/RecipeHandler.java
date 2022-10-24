package hr.unipu.recipe;

import hr.unipu.plantcomputer.RecipeDTO;
import hr.unipu.ui.UiWindow;
import javafx.application.Platform;
import javafx.util.Duration;

import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.TimeUnit;

/**
 * The `recipe_handler.py` module is in charge of running recipes. It provides a
 * service `/<environment_id>/start_recipe` which takes as input a recipe ID and
 * starts the recipe. It also provides a service `/<environment_id>/stop_recipe`
 * which takes no inputs and stops the currently running recipe. It defines a
 * parameter `current_recipe` which stores the ID of the currently running recipe.
 * It also defines a parameter `current_recipe_start` which stores the UNIX
 * timestamp at which the currently running recipe was started. If no recipe is
 * running, `current_recipe` will be set to an empty string and
 * `current_recipe_start` will be set to 0. There should always be exactly one
 * instance of this module per environment in the system.
 */
public class RecipeHandler {


/**
 * # Create a tuple constant of valid environmental variables
 * # Should these be only environment_variables?
 */
public static List<VariableInfo.EnvironmentVariable> ENVIRONMENTAL_VARIABLES = VariableInfo.environment_variables;
public static List<VariableInfo.RecipeVariable> RECIPE_VARIABLES = VariableInfo.recipe_variables;

public static List<VariableInfo.EnvironmentVariable> VALID_VARIABLES = ENVIRONMENTAL_VARIABLES;

//RECIPE_START = VariableInfo.from_dict(rospy.get_param('/var_types/recipe_variables/recipe_start'))
//RECIPE_END = VariableInfo.from_dict(rospy.get_param('/var_types/recipe_variables/recipe_end'))
public static String RECIPE_START = RECIPE_VARIABLES.get(0).getName();
public static String RECIPE_END = RECIPE_VARIABLES.get(1).getName();


/**
 * # This builds a dictionary of publisher instances using a
 * # "dictionary comprehension" (syntactic sugar for building dictionaries).
 * # The constant has to be declared here because get_message_class
 * # needs to be called after the node is initialized.
 */
public static Map PUBLISHERS = new LinkedHashMap();

public static Map<String, String> RECIPE_INTERPRETERS = Map.of(
        "simple", "interpret_simple_recipe",
        "flexformat", "interpret_flexformat_recipe"
);
    private static RecipeDTO recipe;
    private static Map<String, Double> setpoints;
    private static long now_time;
    private static long start_time;

    /**
 * # Our 'babysitting' class that keeps state for the module.
 * RecipeHandler is a manger for keeping track of the currently running recipe
 *     (if any). It offers threadsafe methods for:
 *     - getting the currently running recipe
 *     - setting the recipe
 *     - clearing the recipe
 *     and other things. It also contains handlers for the start_recipe
 *     and stop_recipe services.
 */
public RecipeHandler() {

//    def __init__(self, server, environment):
//        # We create a lock to ensure threadsafety, since service handlers are
//        # run in a separate thread by ROS.
//            self.lock = RLock()
//    self.env_data_db = server[ENVIRONMENTAL_DATA_POINT]
//    self.recipe_db = server[RECIPE]
//    self.environment = environment
//    self.__start_time = None
//    self.__recipe = None

}

/**
 * get_recipe(self):
 */
public static void get_recipe(String recipeName) {

    recipe = RecipeInterpreter.loadFlexFormatRecipe(recipeName);

}


/**
 * get_state(self):
 *
 * Get the state-related variables of the currently running recipe
 */
public static void get_state() {
//    now_time = rospy.get_time()
//    start_time = self.__start_time or now_time
//    return self.get_recipe(), start_time, now_time

    Calendar currentTime = Calendar.getInstance();
    SimpleDateFormat dateFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm");
    dateFormatter.setTimeZone(TimeZone.getTimeZone("Europe/Berlin"));
    Date now_time_date = currentTime.getTime();

    //Date now_time_date = new SimpleDateFormat( "yyyy-MM-dd HH:mm" ).parse( "2017-04-18 20:00" );
    //Date start_time_date = new SimpleDateFormat("yyyy-MM-dd HH:mm").parse( "2017-04-17 14:00" );

    now_time = RecipeInterpreter.unix_time_seconds(now_time_date);

    // If start_time is not defined from previous recipe.
    if (start_time == 0) {
        start_time = now_time - 1;  // Test for recipe in process at first change (1 secs in)
    }

}


/**
 * set_recipe(self, recipe, start_time):
 *
 * Set the currently running recipe... this is the CouchDB recipe document.
 */


/**
 * clear_recipe(self):
 */


/**
 * start_recipe_service(self, data, start_time=None):
 */


/**
 * stop_recipe_service(self, data):
 */


/**
 * register_services(self):
 */


/**
 * recover_any_previous_recipe(self):
 */


/**
 * save_recipe_dp(self, variable):
 *
 * Save the recipe start/end to the env. data pt. DB, so we can restart
 * the recipe if necessary.
 */


/**
 * Our ROS node main entry point.  Starts up the node and then waits forever.
 * @param recipeName
 */
public static void run_recipe(String recipeName) {

    // Initialization of: timezone; static variables date_min, date_max;
    RecipeInterpreter.init();
    
    //TODO Specify local db.
    // - read_environment_from_ns()
    // - recover_any_previous_recipe()

    // Subscribe to our own 'recipe_end' message, so we can stop publishing
    // and clear the recipe when we get it.
    // - clear_recipe()

    // Set rate for WHILE LOOP:
    //      rate_hz = rospy.get_param('~rate_hz', 1)
    //      rate = rospy.Rate(rate_hz)
    TimeUnit samplingRate = TimeUnit.SECONDS;
    long timeToSleep = 10L;     // Sampling rate for recipe state update.


    // WHILE LOOP:
    // - Get current recipe state;
    //      recipe_handler.get_state()
    RecipeHandler.get_state();
    RecipeHandler.get_recipe(recipeName);

    // - If we have a recipe, process it. Running a recipe is a blocking
    //   operation, so the recipe will stay in this turn of the loop
    //   until it is finished.
    // - Interpret recipe:
    //      interpret_recipe = RECIPE_INTERPRETERS[recipe_doc["format"]]
    // - Get recipe state and publish it:
    //      setpoints = interpret_recipe(recipe_doc, start_time, now_time)

    System.out.println("start_time = " + start_time + " now_time = " + now_time);
    setpoints = RecipeInterpreter.interpret_flexformat_recipe(recipe, start_time , now_time);

    // Calculate recipe duration:
    List<RecipeDTO.Phases> phases = recipe.getPhases();
    List<List<Long>> duration_of_phases_steps = RecipeInterpreter.calc_duration_of_phases_steps(phases);
    System.out.println("duration_of_phases_steps: " + duration_of_phases_steps);
    Long recipe_duration = 0L;
    for (List<Long> duration_of_phases_step : duration_of_phases_steps) {
        recipe_duration += duration_of_phases_step.get(0);
    }
    System.out.println("Recipe duration: " + recipe_duration);
    Duration countDownDuration = Duration.hours(recipe_duration.doubleValue());

    Platform.runLater(() -> {
        // Update UI thread from here (if needed).
        UiWindow.init(countDownDuration);
        UiWindow.countdownTile.setVisible(true);
        UiWindow.timer2.start();
    });


    // WHILE LOOP
    ArduinoHandler arduinoHandler = new ArduinoHandler();
    while (UiWindow.isRecipeSelected()) {

        System.out.println("In th WHILE LOOP");

        // FOR LOOP:
        // - Publish any setpoints that we can:
        //      for variable, value in setpoints:
        //          pub = PUBLISHERS[variable]
        //          trace("recipe_handler publish: %s, %s", variable, value)
        //          if variable == RECIPE_END.name:
        //              trace("recipe_handler publish: END!")
        //              # Write an env. data pt. for when we stopped this recipe.
        //              recipe_handler.save_recipe_dp(variable)
        //          elif variable == RECIPE_START.name:
        //              # Write an env. data pt. for when we started this recipe.
        //              recipe_handler.save_recipe_dp(variable)
        //          pub.publish(value)
        //

        // FOR LOOP
        for (Map.Entry<String, Double> entry : setpoints.entrySet()) {

            System.out.println(entry.getKey() + " : " + entry.getValue());

            //Send desired setpoints to ArduinoHandler
            try {
                arduinoHandler.set_desired_setpoint(entry);
            } catch (Exception exception) {
                System.out.println(exception.getMessage());
            }

        }


        // Sleep in WHILE LOOP (before new time step).
        //      rate.sleep()
        try {
            System.out.println("Going to sleep for "
                    + timeToSleep
                    + " seconds");

            samplingRate.sleep(timeToSleep);

            System.out.println("Slept for "
                    + timeToSleep
                    + " seconds");

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    System.out.println("Out of WHILE LOOP");

}

}
