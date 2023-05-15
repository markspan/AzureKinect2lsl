#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <lsl_cpp.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include "BodyTrackingHelpers.h"

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

int main()
{
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED; // K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;

    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;


    lsl_streaminfo info = NULL;

    if (k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker) != K4A_RESULT_SUCCEEDED) {
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", "Body tracker initialization failed!", "AzureKinect2lsl.cpp", __FUNCTION__, 36);
        k4abt_tracker_t tracker = NULL;
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
        printf("Running tracker is standard (slow) mode\n");
        info = lsl_create_streaminfo("Azure-Kinect", "MoCap", 32 * 7, 4, cft_double64, "325wqer4354");
    }
    else
    {
        printf("Running tracker is CUDA mode\n");
        info = lsl_create_streaminfo("Azure-Kinect", "MoCap", 32 * 7, 10, cft_double64, "325wqer4354");
    }


    /* add some meta-data fields to it */
    /* (for more standard fields, see https://github.com/sccn/xdf/wiki/Meta-Data) */
    lsl_xml_ptr desc = lsl_get_desc(info);
    lsl_append_child_value(desc, "manufacturer", "University of Groningen");
    lsl_append_child_value(desc, "model", "Azure Kinect");
    lsl_xml_ptr chns = lsl_append_child(desc, "channels");
    lsl_append_child_value(desc, "unit", "mm");

    for (std::unordered_map<k4abt_joint_id_t, std::string>::const_iterator it = g_jointNames.begin(); it != g_jointNames.end(); it++)
    {
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_posx").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_posy").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_posz").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_oriw").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_orix").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_oriy").c_str());
        lsl_append_child(chns, (std::string(it->second.c_str()) + "_oriz").c_str());
    }
    lsl_outlet outlet = lsl_create_outlet(info, 0, 60);
    do printf("Waiting for recorder\n");
    while (!lsl_wait_for_consumers(outlet, 1200));
    printf("Now sending data...\n");

    int frame_count = 0;
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            //frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }
            float data[7 * 31];
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing
                // TODO: This code simply ignores the bodies if there are more then one.  

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                if (num_bodies > 1) 
                {
                    printf("%zu bodies are detected!\n", num_bodies);
                    return -1;
                }
                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    int j = 0;
                    for (std::unordered_map<k4abt_joint_id_t, std::string>::const_iterator it = g_jointNames.begin(); it != g_jointNames.end(); ++it)
                    {
                        k4a_float3_t     position    = skeleton.joints[it->first].position;
						k4a_quaternion_t orientation = skeleton.joints[it->first].orientation;

                        data[(j * 7)] = position.xyz.x;
                        data[1 + (j * 7)] = position.xyz.y;
                        data[2 + (j * 7)] = position.xyz.z;
                        data[3 + (j * 7)] = orientation.wxyz.w;
                        data[4 + (j * 7)] = orientation.wxyz.x;
                        data[5 + (j * 7)] = orientation.wxyz.y;
                        data[6 + (j * 7)] = orientation.wxyz.z;
                        j = j + 1;
                    }
                }
                lsl_push_sample_f(outlet, data);
                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (frame_count < 100);

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}