/*
 * drivers/cpufreq/cpufreq_governor.c
 *
 * CPUFREQ governors common code
 *
 * Copyright	(C) 2001 Russell King
 *		(C) 2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *		(C) 2003 Jun Nakajima <jun.nakajima@intel.com>
 *		(C) 2009 Alexander Clouter <alex@digriz.org.uk>
 *		(c) 2012 Viresh Kumar <viresh.kumar@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <asm/cputime.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "cpufreq_governor.h"

void dbs_check_cpu(struct dbs_data *dbs_data, int cpu)
{
	struct cpu_dbs_common_info *cdbs = dbs_data->get_cpu_cdbs(cpu);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	struct ex_dbs_tuners *ex_tuners = dbs_data->tuners;
	struct cpufreq_policy *policy;
	unsigned int max_load = 0;
	unsigned int ignore_nice_load;
	unsigned int j;

	if (dbs_data->governor == GOV_ONDEMAND) {
		ignore_nice_load = od_tuners->ignore_nice_load;
	} else if (dbs_data->governor == GOV_ELEMENTALX) {
		ignore_nice_load = ex_tuners->ignore_nice_load;
	} else {
		ignore_nice_load = cs_tuners->ignore_nice_load;
	}

	policy = cdbs->cur_policy;

	/* Get Absolute Load (in terms of freq for ondemand gov) */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_common_info *j_cdbs;
		u64 cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		unsigned int load;
		int io_busy = 0;

		j_cdbs = dbs_data->get_cpu_cdbs(j);

		/*
		 * For the purpose of ondemand, waiting for disk IO is
		 * an indication that you're performance critical, and
		 * not that the system is actually idle. So do not add
		 * the iowait time to the cpu idle time.
		 */
		if (dbs_data->governor == GOV_ONDEMAND)
			io_busy = od_tuners->io_is_busy;
		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, io_busy);

		wall_time = (unsigned int)
			(cur_wall_time - j_cdbs->prev_cpu_wall);
		j_cdbs->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_cdbs->prev_cpu_idle);
		j_cdbs->prev_cpu_idle = cur_idle_time;

		if (ignore_nice_load) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 cdbs->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			cdbs->prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (dbs_data->governor == GOV_ONDEMAND) {
			int freq_avg = __cpufreq_driver_getavg(policy, j);
			if (freq_avg <= 0)
				freq_avg = policy->cur;

			load *= freq_avg;
		}

		if (load > max_load)
			max_load = load;
	}

	dbs_data->gov_check_cpu(cpu, max_load);
}
EXPORT_SYMBOL_GPL(dbs_check_cpu);

static inline void dbs_timer_init(struct dbs_data *dbs_data,
		struct cpu_dbs_common_info *cdbs, unsigned int sampling_rate)
{
	int delay = delay_for_sampling_rate(sampling_rate);

	INIT_DELAYED_WORK_DEFERRABLE(&cdbs->work, dbs_data->gov_dbs_timer);
	schedule_delayed_work_on(cdbs->cpu, &cdbs->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_common_info *cdbs)
{
	cancel_delayed_work_sync(&cdbs->work);
}

int cpufreq_governor_dbs(struct dbs_data *dbs_data,
		struct cpufreq_policy *policy, unsigned int event)
{
	struct od_cpu_dbs_info_s *od_dbs_info = NULL;
	struct cs_cpu_dbs_info_s *cs_dbs_info = NULL;
	struct ex_cpu_dbs_info_s *ex_dbs_info = NULL;
	struct od_ops *od_ops = NULL;
	struct cs_ops *cs_ops = NULL;
	struct ex_ops *ex_ops = NULL;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	struct ex_dbs_tuners *ex_tuners = NULL;
	struct cpu_dbs_common_info *cpu_cdbs;
	unsigned int *sampling_rate, latency, ignore_nice_load, j, cpu = policy->cpu;
	int io_busy = 0;
	int rc;

	cpu_cdbs = dbs_data->get_cpu_cdbs(cpu);

	if (dbs_data->governor == GOV_CONSERVATIVE) {
		cs_dbs_info = dbs_data->get_cpu_dbs_info_s(cpu);
		sampling_rate = &cs_tuners->sampling_rate;
		ignore_nice_load = cs_tuners->ignore_nice_load;
		cs_ops = dbs_data->gov_ops;
	} else if (dbs_data->governor == GOV_ELEMENTALX) {
		ex_tuners = dbs_data->tuners;
		ex_dbs_info = dbs_data->get_cpu_dbs_info_s(cpu);
		sampling_rate = ex_tuners->sampling_rate;
		ignore_nice_load = ex_tuners->ignore_nice_load;
	} else {
		od_dbs_info = dbs_data->get_cpu_dbs_info_s(cpu);
		sampling_rate = &od_tuners->sampling_rate;
		ignore_nice_load = od_tuners->ignore_nice_load;
		od_ops = dbs_data->gov_ops;
		io_busy = od_tuners->io_is_busy;
	}

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!policy->cur)
			return -EINVAL;

		mutex_lock(&dbs_data->mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_common_info *j_cdbs =
				dbs_data->get_cpu_cdbs(j);

			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
					       &j_cdbs->prev_cpu_wall, io_busy);
			if (ignore_nice_load)
				j_cdbs->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}

		if (!policy->governor->initialized) {
			rc = sysfs_create_group(cpufreq_global_kobject,
					dbs_data->attr_group);
			if (rc) {
				mutex_unlock(&dbs_data->mutex);
				return rc;
			}
		}

		if (dbs_data->governor == GOV_CONSERVATIVE) {
			cs_dbs_info->down_skip = 0;
			cs_dbs_info->enable = 1;
			cs_dbs_info->requested_freq = policy->cur;

			if (!policy->governor->initialized) {
				cpufreq_register_notifier(cs_ops->notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);

				dbs_data->min_sampling_rate =
					MIN_SAMPLING_RATE_RATIO *
					jiffies_to_usecs(10);
			}
		} else if (dbs_data->governor == GOV_ELEMENTALX) {
			ex_dbs_info->enable = 1;
		} else {
			od_dbs_info->rate_mult = 1;
			od_dbs_info->sample_type = OD_NORMAL_SAMPLE;
			od_ops->powersave_bias_init_cpu(cpu);

			if (!policy->governor->initialized)
				od_tuners->io_is_busy = od_ops->io_busy();
		}

		if (policy->governor->initialized)
			goto unlock;

		/* policy latency is in nS. Convert it to uS first */
		latency = policy->cpuinfo.transition_latency / 1000;
		if (latency == 0)
			latency = 1;

		/* Bring kernel and HW constraints together */
		dbs_data->min_sampling_rate = max(dbs_data->min_sampling_rate,
				MIN_LATENCY_MULTIPLIER * latency);
		*sampling_rate = max(dbs_data->min_sampling_rate, latency *
				LATENCY_MULTIPLIER);
unlock:
		mutex_unlock(&dbs_data->mutex);

		mutex_init(&cpu_cdbs->timer_mutex);
		dbs_timer_init(dbs_data, cpu_cdbs, *sampling_rate);
		break;

	case CPUFREQ_GOV_STOP:
		if (dbs_data->governor == GOV_CONSERVATIVE)
			cs_dbs_info->enable = 0;

		if (dbs_data->governor == GOV_ELEMENTALX)
			ex_dbs_info->enable = 0;

		dbs_timer_exit(cpu_cdbs);

		mutex_lock(&dbs_data->mutex);
		mutex_destroy(&cpu_cdbs->timer_mutex);
		cpu_cdbs->cur_policy = NULL;

		if (policy->governor->initialized == 1) {
			sysfs_remove_group(cpufreq_global_kobject,
					dbs_data->attr_group);
			if (dbs_data->governor == GOV_CONSERVATIVE)
				cpufreq_unregister_notifier(cs_ops->notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_data->mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&cpu_cdbs->timer_mutex);
		if (policy->max < cpu_cdbs->cur_policy->cur)
			__cpufreq_driver_target(cpu_cdbs->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > cpu_cdbs->cur_policy->cur)
			__cpufreq_driver_target(cpu_cdbs->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		dbs_check_cpu(dbs_data, cpu);
		mutex_unlock(&cpu_cdbs->timer_mutex);
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cpufreq_governor_dbs);
