#include "Copter.h"
#include "mode.h"
#include <AC_PID/AC_PID.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>


# define _VEL_Z_P                    5.0f    // vertical velocity controller P gain default
# define _VEL_Z_I                    1.0f    // vertical acceleration controller I gain default
# define _VEL_Z_IMAX                1000.0f // horizontal velocity controller IMAX gain default
# define _VEL_Z_FILT_HZ             5.0f    // horizontal velocity controller input filter

# define _VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
# define _VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
# define _VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
# define _VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter

# define VEL_DT                   0.0025f

uint32_t last_loop;

float p_target;
float r_target;
float thr_target;

Vector3f __acceleration;
Vector3f final_desired_velocity;
Vector3f v_err;
Vector3f v_ctrl;
Vector3f u_d;
Vector3f grav;
Vector3f v_sum;

AC_PID pid_vel_x(_VEL_XY_P, _VEL_XY_I, 0.0f, _VEL_XY_IMAX, _VEL_XY_FILT_HZ, VEL_DT);
AC_PID pid_vel_y(_VEL_XY_P, _VEL_XY_I, 0.0f, _VEL_XY_IMAX, _VEL_XY_FILT_HZ, VEL_DT);
AC_PID pid_vel_z(_VEL_Z_P, _VEL_Z_I, 0.0f, _VEL_Z_IMAX, _VEL_Z_FILT_HZ, VEL_DT);

void Copter::VelocityControl::cal_vel(Vector3f velocity, float time)
{
	v_sum = velocity + v_sum;
	final_desired_velocity.x = v_sum.x*time;
	final_desired_velocity.y = v_sum.y*time;
	final_desired_velocity.z = v_sum.z*time;
	return;
}

void Copter::VelocityControl::convert_roll_pitch_to_desired_acceleration(float roll_in, float pitch_in, float acc_mag)
{
	Vector3f desired_accel;
	desired_accel.x = -1*sinf(pitch_in);
	desired_accel.y = -1*sinf(roll_in)*cosf(pitch_in);
	desired_accel.z = cosf(roll_in)*cosf(pitch_in);
	__acceleration *= acc_mag;
	return;
}

void Copter::VelocityControl::accel_to_angles(Vector3f desired_ud)
{
	p_target = atanf(-desired_ud.x/desired_ud.z)*(18000.0f/M_PI);
	float cos_p_target = cosf(p_target*M_PI/18000.0f);
	r_target = atanf(desired_ud.y*cos_p_target/desired_ud.z)*(18000.0f/M_PI);
	thr_target = sqrtf(powf(desired_ud.x, 2.0f) + powf(desired_ud.y, 2.0f) + powf(desired_ud.z,2.0f));
}
bool Copter::VelocityControl::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }

    return true;
}

void Copter::VelocityControl::run()
{

	float target_roll, target_pitch;
	float target_yaw_rate;
	float pilot_throttle_scaled;
    float ekfGndSpdLimit, ekfNavVelGainScaler;
    AP::ahrs_navekf().getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

	// if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();

    }
    return;

    // clear landing flag
    set_land_complete(false);
    // delay for integral and derivative

	uint32_t now = AP_HAL::millis();

	float dt = (now - last_loop)*0.001f;

	// sanity check dt
	if (dt >= 0.2f) {
		dt = 0.0f;
	}

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // Convert Roll, Pitch, Yaw Rate input into acceleration
    convert_roll_pitch_to_desired_acceleration(target_roll, target_pitch, pilot_throttle_scaled);

    // calculate velocity
    cal_vel(final_desired_velocity, dt);

    // calculate v_err
    Vector3f c_pos = inertial_nav.get_velocity();
    v_err.x = final_desired_velocity.x - c_pos.x;
    v_err.y = final_desired_velocity.y - c_pos.y;
    v_err.z = final_desired_velocity.z - c_pos.z;

    // set dt
    pid_vel_x.set_dt(dt);
    pid_vel_y.set_dt(dt);
    pid_vel_z.set_dt(dt);

    // calculate PI values
    pid_vel_x.set_input_filter_all(v_err.x);
    pid_vel_y.set_input_filter_all(v_err.y);
    pid_vel_z.set_input_filter_all(v_err.z);

	// get p
	v_ctrl.x = pid_vel_x.get_pi() * ekfNavVelGainScaler;
	v_ctrl.y = pid_vel_y.get_pi() * ekfNavVelGainScaler;
	v_ctrl.z = pid_vel_z.get_pi() * ekfNavVelGainScaler;

	// get gravity vector
    grav.x = 0;
    grav.y = 0;
    grav.z = 981.0f;

    // calculate u_d
    u_d = __acceleration + v_ctrl + grav;

    last_loop = now;

    accel_to_angles(u_d);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(r_target, p_target, target_yaw_rate);

    attitude_control->set_throttle_out(thr_target, true, g.throttle_filt);

}
