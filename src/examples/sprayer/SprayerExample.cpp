/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "SprayerExample.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

SprayerExample::SprayerExample() :
        ModuleParams(nullptr),
        ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) {
}

SprayerExample::~SprayerExample() {
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool SprayerExample::init() {
    ScheduleOnInterval(500_ms);

    return true;
}

void SprayerExample::Run() {
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);


    // DO WORK


    // Example
    // grab latest gyroscope data
    _sensor_gyro_sub.update();
    const sensor_gyro_s &gyro = _sensor_gyro_sub.get();
    PX4_INFO("Gyro x,y,z: \t%8.4f\t%8.4f\t%8.4f\n", (double) gyro.x, (double) gyro.y, (double) gyro.z);

    _airspeed_validated_sub.update();
    const airspeed_validated_s &airspeed_validated = _airspeed_validated_sub.get();
    PX4_INFO("Airspeed validated groundspeed: \t%8.4f\n", (double) airspeed_validated.true_ground_minus_wind_m_s);

    _vehicle_local_position_sub.update();
    const vehicle_local_position_s &vehicle_local_position = _vehicle_local_position_sub.get();
    double groundspeed = sqrt(vehicle_local_position.vx * vehicle_local_position.vx +
                              vehicle_local_position.vy * vehicle_local_position.vy);
    PX4_INFO("Vehicle local position groundspeed: \t%8.4f\n", groundspeed);


    // Example
    // publish some data
    orb_test_s data{};
    data.timestamp = hrt_absolute_time();
    data.val = gyro.device_id;
    _orb_test_pub.publish(data);


//    // Example
//    // grab latest accelerometer data
//    _sensor_accel_sub.update();
//    const sensor_accel_s &accel = _sensor_accel_sub.get();
//
//
//    // Example
//    // publish some data
//    orb_test_s data{};
//    data.timestamp = hrt_absolute_time();
//    data.val = accel.device_id;
//    _orb_test_pub.publish(data);


    perf_end(_loop_perf);
}

int SprayerExample::task_spawn(int argc, char *argv[]) {
    SprayerExample *instance = new SprayerExample();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int SprayerExample::print_status() {
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int SprayerExample::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int SprayerExample::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sprayer_example", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int sprayer_example_main(int argc, char *argv[]) {
    return SprayerExample::main(argc, argv);
}
