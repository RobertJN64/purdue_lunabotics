#include "actuation.h"

namespace actuation
{

	static StepperDir lead_screw_dir;
	static uint8_t lead_screw_en;

	Stepper lead_screw_stepper(actuation_cfg.lead_screw.steps,
							   actuation_cfg.lead_screw.DIR1_pin,
							   actuation_cfg.lead_screw.DIR2_pin);

	void lin_act_hall_cb()
	{
		if (digitalRead(LIN_ACT_HALL_PIN) == LOW)
		{
			lin_act_hall.lin_act_state = static_cast<LinActState>((lin_act_hall.lin_act_state + 1) % LinActState::CNT);
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
			lin_act_hall.lim = AT_LIMIT;
		}
		else
		{
			lin_act_hall.lim = FREE;
		}
	}

	// Lead Screw Actuation

	void lead_screw_hall_cb()
	{
		if (digitalRead(LEAD_SCREW_HALL_PIN) == LOW)
		{
			lead_screw_hall.lead_screw_state = static_cast<LeadScrewState>((lead_screw_hall.lead_screw_state + 1) % LeadScrewState::CNT);
			stepper_off(actuation_cfg.lead_screw);
			lead_screw_hall.lim = AT_LIMIT;
		}
		else
		{
			lead_screw_hall.lim = FREE;
		}
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(actuation_cfg.left_lin_act);
		init_motor(actuation_cfg.right_lin_act);
		init_stepper(actuation_cfg.lead_screw, &lead_screw_stepper);
		stepper_off(actuation_cfg.lead_screw);
		lead_screw_en = 0;
		lead_screw_hall = {.lead_screw_state = LeadScrewState::STORED, .lim = AT_LIMIT};
		lin_act_hall = {.lin_act_state = LinActState::STORED, .lim = AT_LIMIT};
		init_hall(LEAD_SCREW_HALL_PIN, lead_screw_hall_cb);
		init_hall(LIN_ACT_HALL_PIN, lin_act_hall_cb);
	}

	void stepper_step()
	{
		if (lead_screw_en)
		{
			stepper_step(actuation_cfg.lead_screw, &lead_screw_stepper, lead_screw_dir);
		}
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle *nh)
	{
		lead_screw_dir = (actuation.lead_screw > 0) ? EXTEND : RETRACT;
		lead_screw_en = actuation.lead_screw != 0;

		MotorDir angle_dir = (actuation.angle > 0) ? CW : CCW;

		if (lin_act_hall.lim == AT_LIMIT)
		{
			if (lin_act_hall.lin_act_state == LinActState::STORED && angle_dir == CCW)
			{
				return;
			}
			else if (lin_act_hall.lin_act_state == LinActState::FULL_EXT && angle_dir == CW)
			{
				return;
			}
		}

		if (lead_screw_hall.lim == AT_LIMIT)
		{
			if (lead_screw_hall.lead_screw_state == LeadScrewState::STORED && lead_screw_dir == RETRACT)
			{
				return;
			}
			else if (lead_screw_hall.lead_screw_state == LeadScrewState::FULL_EXT && lead_screw_dir == EXTEND)
			{
				return;
			}
		}

		if (actuation.angle != 0)
		{
			write_motor(actuation_cfg.left_lin_act,
						actuation_cfg.left_lin_act.MAX_PWM, angle_dir);
			write_motor(actuation_cfg.right_lin_act,
						actuation_cfg.right_lin_act.MAX_PWM, angle_dir);
		}
		else
		{
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}

		// nh.logerror("lead screw:");
		if (lead_screw_en)
		{
			// nh.logerror("ON");
			stepper_on(actuation_cfg.lead_screw);
		}
		else
		{
			// nh.logerror("STOP");
			stepper_off(actuation_cfg.lead_screw);
		}
	}

}