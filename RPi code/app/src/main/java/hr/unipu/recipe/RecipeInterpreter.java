package hr.unipu.recipe;

import com.fasterxml.jackson.databind.ObjectMapper;
import hr.unipu.plantcomputer.RecipeDTO;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.file.Paths;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.time.*;
import java.util.*;
import java.util.concurrent.TimeUnit;

/**
 * Example:
 *
 * RecipeInterpreter.init();
 * FlexFormatDTO recipe = RecipeInterpreter.loadFlexFormatRecipe("MOCK_RECIPE_FLEXFORMAT_A.json");
 * long now_time = RecipeInterpreter.unix_time_seconds(now_time_date);*
 * long start_time = RecipeInterpreter.unix_time_seconds(start_time_date);
 * Map setpoints = RecipeInterpreter.interpret_flexformat_recipe(recipe, start_time , now_time);
 *
 */
public class RecipeInterpreter {

    // A threshold to compare time values in seconds.
    private static Integer THRESHOLD = 1;

    // Number of millisecond from EPOCH (normally 1/01/1970 00:00 UTC)
    public static Date EPOCH = Date.from(Instant.ofEpochMilli(0));

    public static long MIN_DATE;    // For verifying time format.
    public static long MAX_DATE;


    /**
     * Initialization of:
     * - timezone;
     * - static variables;
     */
    public static void init () {
        TimeZone.setDefault(TimeZone.getTimeZone("UTC"));

        try {
            Date date_min = new SimpleDateFormat( "MM-dd-yyyy HH:mm:ss" ).parse( "06-11-2010 07:00:00" );
            MIN_DATE = unix_time_seconds(date_min); //1.276.239.600 (seconds).
            Date date_max = new SimpleDateFormat( "MM-dd-yyyy HH:mm:ss" ).parse( "06-11-2035 07:00:00" );
            MAX_DATE = unix_time_seconds(date_max); //2.065.158.000 (seconds).
        } catch (ParseException e) {
            e.printStackTrace();
        }

    }


    /**
     * def unix_time_seconds(dt):
     *      return (dt - EPOCH).total_seconds()
     * @return
     */
    public static long unix_time_seconds(Date dt) {
        TimeUnit time = TimeUnit.SECONDS;
        return time.toSeconds((dt.getTime() - EPOCH.getTime()) / 1000);
    }


    public static Map loadSimpleRecipe(String fileName) {
        String path = "C:\\Users\\Fibrillator\\Documents\\JAVA-KOTLIN\\MyIoT\\MyCode - 00_Food Computer\\RPi code - Mosquitto Food Computer gateway\\app\\src\\main\\resources\\recipes\\" + fileName;

        ObjectMapper mapper = new ObjectMapper();
        Map<?, ?> recipe = null;
        try {
            recipe = mapper.readValue(Paths.get(path).toFile(), Map.class);
        } catch (IOException e) {
            e.printStackTrace();
        }
        /*
        for (Map.Entry<?, ?> entry : recipe.entrySet()) {
            System.out.println(entry.getKey() + "=" + entry.getValue());
        }
         */

        return recipe;
    }


    public static RecipeDTO loadFlexFormatRecipe(String fileName) {
        String path = "C:\\Users\\Fibrillator\\Documents\\JAVA-KOTLIN\\MyIoT\\MyCode - 00_Food Computer\\RPi code - Mosquitto Food Computer gateway\\app\\src\\main\\resources\\recipes\\" + fileName;

        ObjectMapper mapper = new ObjectMapper();
        RecipeDTO recipe = null;
        try {
            recipe = mapper.readValue(Paths.get(path).toFile(), RecipeDTO.class);
            if (recipe.getFormat().equals("phased")) {
                recipe.mapOperations2Phases();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
        /*
        for (Map.Entry<?, ?> entry : recipe.entrySet()) {
            System.out.println(entry.getKey() + "=" + entry.getValue());
        }
         */
        System.out.println("Testing: air_temperature (should be 22) = " + recipe.getPhases().get(0).getStep().getAir_temperature().get(0).getValue());

        return recipe;

    }

    /**
     * interpret_simple_recipe(recipe, start_time, now_time):
     *
     * Produces a tuple of ``(variable, value)`` pairs by building up
     * a recipe state from walking through the recipe keyframes
     */
    public static Map interpret_simple_recipe(Map recipe, long start_time, long now_time) {

        // e.g. 'now_time' should be greater from 'start_time'
        // now_time = (int) (new Date().getTime()/1000);
        // start_time = now_time - 30;

        String _id = (String) recipe.get("_id");
        ArrayList operations = (ArrayList) recipe.get("operations");
        System.out.println("_id = " + _id);
        System.out.println("operations = " + operations.toString());

        int lastIndex = operations.size() - 1;
        ArrayList lastOperation = (ArrayList) operations.get(lastIndex);
        long end_time_relative = (Integer) lastOperation.get(0);
        System.out.println("end_time_relative = " + end_time_relative);
        System.out.printf("recipe_handler: interpret_simple_recipe end_time_relative=%s\n", end_time_relative);

        long end_time = start_time + end_time_relative;

        // If start time is at some point in the future beyond the threshold.
        if (start_time - now_time > THRESHOLD)
            System.err.println("Recipes cannot be scheduled for the future.");

        // If there are no recipe operations, immediately start and stop.
        // The recipe.
        if (operations.size() < 1)
            return Map.of(Map.of("recipe_start", _id), Map.of("recipe_end", _id));
        if (now_time >= (end_time + THRESHOLD))
            return Map.of("recipe_end", _id);
        if (Math.abs(now_time - start_time) < THRESHOLD)
            return Map.of("recipe_start", _id);

        long now_relative = (now_time - start_time);

        // Create a state object to accrue recipe setpoint values.
        Map<String, Double> state = new LinkedHashMap<>();  // Keeps the keys in the order they were inserted.
        // Build up state up until now_time (inclusive).
        System.out.printf("recipe_handler: interpret_simple_recipe now=%s\n", now_relative);
        for (int i=0; i < operations.size(); i++) {
            ArrayList operation = (ArrayList) operations.get(i);
            Integer timestamp = (Integer) operation.get(0);
            String variable = (String) operation.get(1);
            Double value = Double.parseDouble( operation.get(2).toString() );
            if (timestamp > now_relative) break;
            state.put(variable, value);
            System.out.printf("recipe_handler: interpret_simple_recipe: %s %s %s\n", timestamp, variable, value);
        }

        return state;
    }


    /**
     * interpret_flexformat_recipe(recipe, start_time, now_time):
     *
     * Recipe Interpreter should read a recipe, now_time, start_time, variable and return a value.
     * - Determine the time since the beginning of the recipe.
     * - Determine what is the current step
     * - Calculate the remaining time left in this step.
     * - Look up the value within that step for that variable.
     */
    public static Map interpret_flexformat_recipe(RecipeDTO recipe, long start_time, long now_time) {

        String _id = (String) recipe.get_id();
        System.out.println("_id = " + _id);

        List<RecipeDTO.Phases> phases = recipe.getPhases();
        System.out.println("phases = " + phases.toString());
        System.out.println("phases size = " + phases.size());

        verify_time_units(now_time);
        verify_time_units(start_time);

        // If start time is at some point in the future beyond the threshold.
        if (start_time - now_time > THRESHOLD)
            System.err.println("Recipes cannot be scheduled for the future.");

        // If there are no recipe operations, immediately start and stop.
        // The recipe.
        if (phases.size() < 1)
            return Map.of(Map.of("recipe_start", _id), Map.of("recipe_end", _id));
        if (Math.abs(now_time - start_time) < THRESHOLD)
            return Map.of("recipe_start", _id);

        //time_units = verify_time_units_are_consistent(recipe['phases'])
        String time_units = verify_time_units_are_consistent(phases);
        System.out.println("time_units = " + time_units);

        // Returns a list of the phases and step durations:
        // [(duration_of_phase_1, duration_of_step_1), (duration_of_phase_2, duration_of_step_2), etc]
        List<List<Long>> duration_of_phases_steps = calc_duration_of_phases_steps(phases);
        System.out.println("duration_of_phases_steps: " + duration_of_phases_steps);

        Long[] ans = calc_phase_and_time_remaining(duration_of_phases_steps,
                                                    start_time,
                                                    now_time,
                                                    time_units);
        Long current_phase_number = ans[0];
        Long duration_in_step = ans[1];
        System.out.println("current_phase_number: " + current_phase_number);
        System.out.println("duration_in_step: " + duration_in_step);


        // Need to create a function to calculate the end time of the recipe
        // if now_time >= (end_time + THRESHOLD):
        //    return ((RECIPE_END.name, _id),)
        RecipeDTO.Phases current_phase = phases.get(Math.toIntExact(current_phase_number));

        // Create a state object to accrue recipe setpoint values.
        Map<String, Double> state = new LinkedHashMap<>();

        List<List<?>> items = current_phase.getStep().items();

        int size_of_step = items.size();
        System.out.println("size_of_step (in current phase): " + size_of_step);

        Long start_time_variable = 0L;
        Long end_time_variable = 0L;
        Double value = 0.0;
        String variable = "";
        
        //for variable, variable_step_data in current_phase['step'].items():
        for (int i=0; i < size_of_step; i++){
            int size_of_item = items.get(i).size();
            System.out.println("size_of_item (in current step): " + size_of_item);

            List<?> variable_step_data = items.get(i);
            Map<String, Double> result = determine_value_for_step(variable_step_data, duration_in_step);     //state[variable] = value
            state.putAll(result);
        }

        return state;
    }


    /**
     * verify_time_units(time_var):
     *
     * Verifies the units for incoming time variables are valid.
     */
    public static void verify_time_units(long time_var) {
        if (MIN_DATE < time_var && time_var < MAX_DATE)
            return;
        else
            System.err.println("Variable time format is not correct. The value should be between {" + MIN_DATE + "} and {" + MAX_DATE + "}, but received: " + time_var);
    }


    /**
     * verify_time_units_are_consistent(recipe_phases):
     *
     * The time units are stored for each phase in the recipe rather than at the recipe level.
     * Need to verify they all match for now.
     * - Later add ability for them to be different.
     */
    public static String verify_time_units_are_consistent(List<RecipeDTO.Phases> phases) {
        String time_units = "";

        for (RecipeDTO.Phases phase : phases) {
            if (phase.getTime_units().isEmpty())
                System.err.println("time_units is missing from the phase. Please check recipe format");
            if (time_units.equals("")) {
                time_units = phase.getTime_units();
            } else if (!time_units.equals(phase.getTime_units())) {
                System.err.println("time_units are not consistent across each phase in the recipe. {" + time_units + "} != {" + phase.getTime_units() +"}");
            }
        }

        return time_units;
    }


    /**
     * determine_value_for_step(variable_step_data, duration_in_step):
     *
     * variable_step_data = [{"start_time": 0, "end_time": 6, "value": 20},
     *                           {"start_time": 6, "end_time": 18, "value": 23},
     *                           {"start_time": 18, "end_time": 24, "value": 19}]
     * duration_in_step = 15
     * Given the time within a step, what value is expected.
     */
    public static Map<String, Double> determine_value_for_step(List<?> variable_step_data, Long duration_in_step) {

        Long start_time = 0L;
        Long end_time = 0L;
        String variable = "";
        Double value = 0.0;
        Map<String, Double> ans = new LinkedHashMap<>();

        for (int j=0; j < variable_step_data.size(); j++) {
            Object item = variable_step_data.get(j);

            if (item.getClass() == RecipeDTO.Phases.Step.AirTemperature.class) {
                start_time = ((RecipeDTO.Phases.Step.AirTemperature) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.AirTemperature) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.AirTemperature) item).getName();
                value = ((RecipeDTO.Phases.Step.AirTemperature) item).getValue();
            }
            if (item.getClass() == RecipeDTO.Phases.Step.LightIntensityBlue.class) {
                start_time = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getName();
                value  = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getValue();
            }
            if (item.getClass() == RecipeDTO.Phases.Step.LightIntensityRed.class) {
                start_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getName();
                value = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getValue();
            }
            if (item.getClass() == RecipeDTO.Phases.Step.LightIlluminance.class) {
                start_time = ((RecipeDTO.Phases.Step.LightIlluminance) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.LightIlluminance) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.LightIlluminance) item).getName();
                value = ((RecipeDTO.Phases.Step.LightIlluminance) item).getValue();
            }
            if (item.getClass() == RecipeDTO.Phases.Step.Nutrient_FloraDuoA.class) {
                start_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getName();
                value = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getValue();
            }
            if (item.getClass() == RecipeDTO.Phases.Step.Nutrient_FloraDuoB.class) {
                start_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getStart_time();
                end_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getEnd_time();
                variable = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getName();
                value = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getValue();
            }

            if (start_time <= duration_in_step && duration_in_step <= end_time) {
                System.out.println("variable: " + variable + ", value: " + value + ", start_time: " + start_time + ", end_time: " + end_time);
                ans.put(variable, value);
                return ans;
            }

        }

        return ans;
    }


    /**
     * calculate_max_duration_from_step(step):
     *
     * Determines the total duration of this step. Normally it is 24 hours.
     * - Could add other validation steps here as well.
     * - Convert to numpy and use argmax
     */
    public static Long calculate_max_duration_from_step(RecipeDTO.Phases.Step step) {
        Long max_time = 0L;

        Method[] allMethods = step.getClass().getMethods();
        Method[] getMethods = (Method[]) Arrays.stream(allMethods)
                .filter(method -> method.toString().contains("get"))
                .filter(method -> !method.toString().contains("getClass"))
                .toArray(Method[]::new);
//        Arrays.stream(getMethods)
//                .forEach(method -> System.out.println("Filtered methods: " + method.toString()));

        //System.out.println("Number of get methods1: " + getMethods.length);
        //System.out.println("Number of get methods2: " + step.toString().length());

        List<List<?>> items = step.items();
        //System.out.println("Number of items: " + items.size());

        int size_of_step = items.size();
        System.out.println("size_of_step: " + size_of_step);
        Long start_time = 0L;
        Long end_time = 0L;
        for (int i=0; i < size_of_step; i++){
            int size_of_item = items.get(i).size();
            for (int j=0; j < size_of_item; j++) {
                //start_time = step.getAir_temperature().get(i).getStart_time();
                //end_time = step.getAir_temperature().get(i).getEnd_time();
                Object item = items.get(i).get(j);
                if (item.getClass() == RecipeDTO.Phases.Step.AirTemperature.class) {
                    start_time = ((RecipeDTO.Phases.Step.AirTemperature) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.AirTemperature) item).getEnd_time();
                }
                if (item.getClass() == RecipeDTO.Phases.Step.LightIntensityBlue.class) {
                    start_time = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.LightIntensityBlue) item).getEnd_time();
                }
                if (item.getClass() == RecipeDTO.Phases.Step.LightIntensityRed.class) {
                    start_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getEnd_time();
                }
                if (item.getClass() == RecipeDTO.Phases.Step.LightIntensityRed.class) {
                    start_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.LightIntensityRed) item).getEnd_time();
                }
                if (item.getClass() == RecipeDTO.Phases.Step.Nutrient_FloraDuoA.class) {
                    start_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoA) item).getEnd_time();
                }
                if (item.getClass() == RecipeDTO.Phases.Step.Nutrient_FloraDuoB.class) {
                    start_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getStart_time();
                    end_time = ((RecipeDTO.Phases.Step.Nutrient_FloraDuoB) item).getEnd_time();
                }
                //System.out.println("start_time: " + start_time);
                //System.out.println("end_time: " + end_time);

                if (start_time > end_time)
                    System.err.println("Start_time is after end time.");
                else if (max_time < Math.max(start_time, end_time))
                    max_time = Math.max(start_time, end_time);
            }
        }
        System.out.println("max_time = " + max_time);
        return max_time;
    }


    /**
     * calc_duration_of_phases_steps(phases):
     *
     * Returns a list with the duration of the step and the entire phase
     */
    public static List<List<Long>> calc_duration_of_phases_steps(List<RecipeDTO.Phases> phases) {
        List<List<Long>> duration_of_phases_steps = new ArrayList<>() ;

        for (RecipeDTO.Phases phase : phases) {
            Integer cycles = phase.getCycles();
            Long max_duration = calculate_max_duration_from_step(phase.getStep());
            duration_of_phases_steps.add(List.of(cycles * max_duration, max_duration));
        }

        return duration_of_phases_steps;
    }


    /**
     * convert_duration_units(duration, units='hours'):
     *
     * Converts a number duration from Seconds into the units specified in the options(units variable).
     * @return
     */
    public static Long convert_duration_units(Long duration, String units) {

        Map<String, Double> divider = Map.of("hours", 3600.0,
                            "days", 3600*24.0,
                            "milliseconds", 0.001,
                            "ms", 0.001,
                            "seconds", 1.0);
        if ( divider.get("units") == null )
            System.err.println("Error time_units in recipe are not available. Valid options are: days, hours, milliseconds, ms");

        Long duration_in_hours = (long) (duration / divider.get(units));
        return duration_in_hours;
    }


    /**
     * offset_duration_by_time_from_start(start_time):
     *
     * Calculates how many hours are used in the current day, so the times set in the recipe are relative to midnight,
     * not start_time.
     */


    /**
     * calc_phase_and_time_remaining(duration_of_phases_steps, start_time, now_time, time_units):
     *
     * Calculates how far along the recipe is in progress given the start_time and the time it is now (aka now_time).
     * TODO: Add function to determine the starting point: Subtract the hours out of the current day from the elapsed time.
     *
     *     duration_of_phases_steps == [(336, 24), (480, 24), (168, 24)]  #total Hours in phase, hours per step in phase (usually days)
     *     now_time : datetime : Local current time in seconds from EPOCH
     *     start_time : datetime : Local time the recipe started in seconds from EPOCH
     *     return: current_phase_number, duration_in_phase
     */
    public static Long[] calc_phase_and_time_remaining(List<List<Long>> duration_of_phases_steps, Long start_time, Long now_time, String time_units) {

        Long current_phase_number = 0L;
        Long duration_in_phase = 0L;
        Long[] ans = new Long[2];

        Long time_elapsed = now_time - start_time;
        //time_elapsed = time_elapsed - offset_duration_by_time_from_start(start_time)  # Offset elapsed time by hours on first day. this method doesn't work.
        System.out.println("time_elapsed [in seconds]: " + time_elapsed);

        time_elapsed = convert_duration_units(time_elapsed, time_units);
        System.out.println("time_elapsed [in " + time_units + "]: " + time_elapsed);

        for (int i=0; i < duration_of_phases_steps.size(); i++) {
            Long total_duration = duration_of_phases_steps.get(i).get(0);
            Long step_duration =  duration_of_phases_steps.get(i).get(1);
            System.out.println("total_duration: " + total_duration);
            System.out.println("step_duration: " + step_duration);

            if (time_elapsed > total_duration)
                time_elapsed -= total_duration;
            else {
                duration_in_phase = time_elapsed % step_duration;
                current_phase_number = (long) i;
                break;
            }
        }

        ans[0] = current_phase_number;
        ans[1] = duration_in_phase;
        return ans;
    }


}
