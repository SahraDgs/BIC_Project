/* Includes_BEGIN */
#include <math.h>
#include <float.h>
/* Includes_END */

/* Externs_BEGIN */

#define PI 3.14159265358979323846

#define NUMBER_CHOICES 10   /* Change this to your real number of choices */

#define L1 30.2
#define L2 26.9
#define L3 10.6
#define WEIGHT_VAR_ANGLE 0.4

static double deg2rad(double deg)
{
    return deg * PI / 180.0;
}

/* Externs_END */

void system_Start_wrapper(void)
{
/* Start_BEGIN */
/*
 * Custom Start code goes here.
 */
/* Start_END */
}

void system_Outputs_wrapper(const real_T *u0,
                            real_T *y0)
{
/* Output_BEGIN */

    double pos_final[2];
    double current_theta[3];

    double best_cost = DBL_MAX;
    int best_index = -1;

    int i;
    int j;

    pos_final[0] = u0[0];
    pos_final[1] = u0[1];

    current_theta[0] = u0[2];
    current_theta[1] = u0[3];
    current_theta[2] = u0[4];

    for (i = 0; i < NUMBER_CHOICES; i++) {

        int base = 5 + 6*i;

        double theta0_plus  = u0[base + 0];
        double theta1_plus  = u0[base + 1];
        double theta2_plus  = u0[base + 2];

        double theta0_minus = u0[base + 3];
        double theta1_minus = u0[base + 4];
        double theta2_minus = u0[base + 5];

        double angle_l1;
        double angle_l2;
        double angle_l3;

        double x_after_rot;
        double y_after_rot;

        double position_cost;
        double angle_cost;
        double total_cost;

        /*
         * Ignore choices that add and remove the same nonzero angle.
         * This is safer than testing 0 == 0.
         */
        if (
            (theta0_plus != 0.0 && theta0_plus == theta0_minus) ||
            (theta1_plus != 0.0 && theta1_plus == theta1_minus) ||
            (theta2_plus != 0.0 && theta2_plus == theta2_minus)
        ) {
            continue;
        }

        angle_l1 =
            current_theta[0]
            + deg2rad(theta0_plus - theta0_minus);

        angle_l2 =
            current_theta[0] + current_theta[1]
            + deg2rad(theta0_plus + theta1_plus
                    - theta0_minus - theta1_minus);

        angle_l3 =
            current_theta[0] + current_theta[1] + current_theta[2]
            + deg2rad(theta0_plus + theta1_plus + theta2_plus
                    - theta0_minus - theta1_minus - theta2_minus);

        x_after_rot =
            L1 * cos(angle_l1)
            + L2 * cos(angle_l2)
            + L3 * cos(angle_l3);

        y_after_rot =
            L1 * sin(angle_l1)
            + L2 * sin(angle_l2)
            + L3 * sin(angle_l3);

        position_cost =
            (x_after_rot - pos_final[0]) * (x_after_rot - pos_final[0])
            + (y_after_rot - pos_final[1]) * (y_after_rot - pos_final[1]);

        angle_cost =
            WEIGHT_VAR_ANGLE * (
                deg2rad(theta0_plus + theta0_minus) * deg2rad(theta0_plus + theta0_minus)
                + deg2rad(theta1_plus + theta1_minus) * deg2rad(theta1_plus + theta1_minus)
                + deg2rad(theta2_plus + theta2_minus) * deg2rad(theta2_plus + theta2_minus)
            );

        total_cost = position_cost + angle_cost;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_index = i;
        }
    }

    if (best_index >= 0) {
        int base_best = 5 + 6*best_index;

        for (j = 0; j < 6; j++) {
            y0[j] = u0[base_best + j];
        }
    } else {
        for (j = 0; j < 6; j++) {
            y0[j] = 0.0;
        }
    }

/* Output_END */
}

void system_Terminate_wrapper(void)
{
/* Terminate_BEGIN */
/*
 * Custom Terminate code goes here.
 */
/* Terminate_END */
}




























void system_Outputs_wrapper(const real_T *pos_final,
                            const real_T *current_theta_1,
                            const real_T *current_theta_2,
                            const real_T *current_theta_3,
                            real_T *best_next_theta_1,
                            real_T *best_next_theta_2,
                            real_T *best_next_theta_3)
{
/* Output_BEGIN */

    double current_theta[3];

    double best_cost = DBL_MAX;
    int best_index = -1;

    double best_theta_1 = current_theta_1[0];
    double best_theta_2 = current_theta_2[0];
    double best_theta_3 = current_theta_3[0];

    int i;

    current_theta[0] = current_theta_1[0];
    current_theta[1] = current_theta_2[0];
    current_theta[2] = current_theta_3[0];

    for (i = 0; i < NUMBER_CHOICES; i++) {

        /*
         * theta_choices must be declared elsewhere, for example:
         *
         * static const double theta_choices[NUMBER_CHOICES][6] = {
         *     {theta0_plus, theta1_plus, theta2_plus,
         *      theta0_minus, theta1_minus, theta2_minus},
         *     ...
         * };
         */

        double theta0_plus  = theta_choices[i][0];
        double theta1_plus  = theta_choices[i][1];
        double theta2_plus  = theta_choices[i][2];

        double theta0_minus = theta_choices[i][3];
        double theta1_minus = theta_choices[i][4];
        double theta2_minus = theta_choices[i][5];

        double next_theta_1;
        double next_theta_2;
        double next_theta_3;

        double angle_l1;
        double angle_l2;
        double angle_l3;

        double x_after_rot;
        double y_after_rot;

        double position_cost;
        double angle_cost;
        double total_cost;

        /*
         * Ignore choices that add and remove the same nonzero angle.
         */
        if (
            (theta0_plus != 0.0 && theta0_plus == theta0_minus) ||
            (theta1_plus != 0.0 && theta1_plus == theta1_minus) ||
            (theta2_plus != 0.0 && theta2_plus == theta2_minus)
        ) {
            continue;
        }

        next_theta_1 =
            current_theta[0]
            + deg2rad(theta0_plus - theta0_minus);

        next_theta_2 =
            current_theta[1]
            + deg2rad(theta1_plus - theta1_minus);

        next_theta_3 =
            current_theta[2]
            + deg2rad(theta2_plus - theta2_minus);

        angle_l1 = next_theta_1;

        angle_l2 = next_theta_1 + next_theta_2;

        angle_l3 = next_theta_1 + next_theta_2 + next_theta_3;

        x_after_rot =
            L1 * cos(angle_l1)
            + L2 * cos(angle_l2)
            + L3 * cos(angle_l3);

        y_after_rot =
            L1 * sin(angle_l1)
            + L2 * sin(angle_l2)
            + L3 * sin(angle_l3);

        position_cost =
            (x_after_rot - pos_final[0]) * (x_after_rot - pos_final[0])
            + (y_after_rot - pos_final[1]) * (y_after_rot - pos_final[1]);

        angle_cost =
            WEIGHT_VAR_ANGLE * (
                deg2rad(theta0_plus + theta0_minus) * deg2rad(theta0_plus + theta0_minus)
                + deg2rad(theta1_plus + theta1_minus) * deg2rad(theta1_plus + theta1_minus)
                + deg2rad(theta2_plus + theta2_minus) * deg2rad(theta2_plus + theta2_minus)
            );

        total_cost = position_cost + angle_cost;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_index = i;

            best_theta_1 = next_theta_1;
            best_theta_2 = next_theta_2;
            best_theta_3 = next_theta_3;
        }
    }

    if (best_index >= 0) {
        best_next_theta_1[0] = best_theta_1;
        best_next_theta_2[0] = best_theta_2;
        best_next_theta_3[0] = best_theta_3;
    } else {
        best_next_theta_1[0] = current_theta_1[0];
        best_next_theta_2[0] = current_theta_2[0];
        best_next_theta_3[0] = current_theta_3[0];
    }

/* Output_END */
}