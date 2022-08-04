package hr.unipu.recipe;

import java.util.List;

/**
 * The `pid.py` module is a Python implementation of a
 * Proportional-Integral-Derivative controller for ROS. By default, it listens on
 * a topic "desired" for the current set point and a topic "state" for the current
 * state of the plant being controller. It then writes to a topic "cmd" with the
 * output of the PID controller. If the parameter `variable` is defined, these
 * topics will be renamed as follows. This makes it easy to integrate this PID
 * controller with ROS topics from firmware modules without remapping each of the
 * topics individually.
 *     desired -> <variable>/desired
 *     state -> <variable>/measured
 *     cmd -> <variable>/commanded
 * It also reads configuration from a number of other ROS parameters as well. The
 * controller gains are passed in as parameters `Kp`, `Ki`, and `Kd`. It also
 * accepts an `upper_limit` and `lower_limit` to bound the control effort output.
 * `windup_limit` defines a limit for the integrator of the control loop.
 * `deadband_width` can be used to apply a deadband to the control effors.
 * Specifically, commands with absolute value less than `deadband_width` will be
 * changed to 0.
 */
public class ControllerPID {
    private Double Kp;
    private Double Ki;
    private Double Kd;
    private Double upper_limit;
    private Double lower_limit;
    private Double windup_limit;
    private Double deadband_width;

    private Double set_point;
    private Double last_error;
    private Double integrator;

    private String variableName;

    public ControllerPID() {
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        upper_limit = 1.0;
        lower_limit = 1.0;
        windup_limit = 1000.0;
        deadband_width = 0.0;
        variableName = "";

        List param_names = List.of("Kp", "Ki", "Kd", "lower_limit", "upper_limit", "windup_limit", "deadband_width");

        String sub_name = "cmd";
        String state_sub_name = "state";
        String desired_sub_name = "desired";

        /**
         *   variable = rospy.get_param("~variable", None)
         *     if variable is not None:
         *         pub_name = "{}/commanded".format(variable)
         *         state_sub_name = "{}/measured".format(variable)
         *         desired_sub_name = "{}/desired".format(variable)
         *
         *     pub = rospy.Publisher(pub_name, Float64, queue_size=10)
         */

    }

    public ControllerPID(Double Kp, Double Ki, Double Kd, Double upper_limit, Double lower_limit, Double windup_limit, Double deadband_width, String variableName) {

        this(); // Call no-name constructor.

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.upper_limit = upper_limit;
        this.lower_limit = lower_limit;
        this.windup_limit = windup_limit;
        this.deadband_width = deadband_width;

        this.set_point = null;
        this.last_error = 0.0;
        this.integrator = 0.0;

        this.variableName = variableName;
    }


    public Double update(Double state) {

         // If setpoint was made null, or was already null, do nothing.
        if (state == null) return null;

        Double error = this.set_point - state;

        if (Math.abs(error) < deadband_width)
            return 0.0;

        Double p_value = this.Kp * error;
        Double d_value = this.Kd * (error - this.last_error);
        this.last_error = error;
        this.integrator = this.integrator + error;
        this.integrator = Math.max(-this.windup_limit, Math.min(this.windup_limit, this.integrator));
        Double i_value = this.Ki * this.integrator;

        Double res = p_value + i_value + d_value;
        res = Math.min(this.upper_limit, Math.max(this.lower_limit, res));

        return res;
    }


    public void state_callback(Double state) {
        Double cmd = this.update(state);
        if (cmd == null) {
            return;
        }

    }


    public void set_point_callback(Double set_point) {
        this.set_point = set_point;
    }

    /**
     * # When we receive the recipe end message, reset this PID controller to its default values.
     * # This disables the set point so the controller will just idle until it is set by a new recipe.
     */
    public void recipe_end_callback() {
        this.set_point = null;
    }


}
