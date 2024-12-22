/**
 * Azure Kinect to LSL Streamer
 * This program captures skeleton tracking data from an Azure Kinect device, formats it for LSL (Lab Streaming Layer),
 * and sends it as a stream for use in real-time applications. It uses the Azure Kinect SDK and the LSL SDK.
 */

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <unordered_map>
#include <lsl_cpp.h> // Lab Streaming Layer library
#include <k4a/k4a.h> // Azure Kinect SDK
#include <k4abt.h>   // Azure Kinect Body Tracking SDK
#include "BodyTrackingHelpers.h" // Custom helper library for joint names

#define VERIFY(result, error)                                                                            \
    if (result != K4A_RESULT_SUCCEEDED)                                                                  \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }

/**
 * Main function to initialize the Azure Kinect, set up the LSL stream, and send data.
 */
int main()
{
    // Step 1: Open the Azure Kinect device
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Failed to open Azure Kinect device!");

    // Step 2: Configure the device for depth tracking (no color camera required)
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED; // Depth mode: Narrow field of view
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30; // 30 FPS is sufficient for most applications

    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Failed to start cameras on Azure Kinect!");

    // Step 3: Retrieve the device's calibration data
    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
           "Failed to retrieve calibration data!");

    // Step 4: Initialize the body tracker
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA; // Use CUDA for faster processing

    // Step 5: Set up the LSL stream
    lsl_streaminfo info = NULL; // Stream metadata object

    if (k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker) != K4A_RESULT_SUCCEEDED)
    {
        // Fallback to slower CPU processing if CUDA fails
        printf("CUDA tracker initialization failed! Falling back to standard mode.\n");
        tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
        VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Failed to initialize body tracker!");
        info = lsl_create_streaminfo("Azure-Kinect", "MoCap", 32 * 7, 4, cft_double64, "325wqer4354");
    }
    else
    {
        printf("CUDA tracker initialized successfully.\n");
        info = lsl_create_streaminfo("Azure-Kinect", "MoCap", 32 * 7, 10, cft_double64, "325wqer4354");
    }

    // Add metadata to the LSL stream
    lsl_xml_ptr desc = lsl_get_desc(info);
    lsl_append_child_value(desc, "manufacturer", "University of Groningen");
    lsl_append_child_value(desc, "model", "Azure Kinect");

    // Create a 'channels' node to define variables being logged
    lsl_xml_ptr chns = lsl_append_child(desc, "channels");

    // Iterate over joint names and append metadata for each variable
    std::vector<std::string> suffixes = {"_posx", "_posy", "_posz", "_oriw", "_orix", "_oriy", "_oriz"};
    for (std::unordered_map<k4abt_joint_id_t, std::string>::const_iterator it = g_jointNames.begin(); it != g_jointNames.end(); ++it)
    {
        std::string jointName = it->second;
        for (const auto &suffix : suffixes)
        {
            lsl_xml_ptr channel = lsl_append_child(chns, "channel");
            lsl_append_child_value(channel, "name", (jointName + suffix).c_str());
            lsl_append_child_value(channel, "unit", "mm"); // Units in millimeters
        }
    }

    // Create an LSL outlet to send the data stream
    lsl_outlet outlet = lsl_create_outlet(info, 0, 60);

    // Wait for an LSL recorder to connect
    printf("Waiting for LSL recorder...\n");
    while (!lsl_wait_for_consumers(outlet, 1200))
        ;

    printf("Recorder connected. Now sending data...\n");

    // Step 6: Start data processing loop
    int frame_count = 0;
    do
    {
        // Retrieve a frame from the Kinect device
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Queue the frame for body tracking
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Release sensor capture after queuing
            if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                printf("Failed to queue capture for processing.\n");
                break;
            }

            // Retrieve body tracking results
            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Ensure only one body is tracked
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                if (num_bodies > 1)
                {
                    printf("Multiple bodies detected (%zu)! Exiting...\n", num_bodies);
                    return -1;
                }

                // Extract skeleton data
                float data[7 * 31]; // Buffer for skeleton data
                for (std::unordered_map<k4abt_joint_id_t, std::string>::const_iterator it = g_jointNames.begin(); it != g_jointNames.end(); ++it)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

                    k4a_float3_t position = skeleton.joints[it->first].position;
                    k4a_quaternion_t orientation = skeleton.joints[it->first].orientation;

                    int j = std::distance(g_jointNames.begin(), it);
                    data[j * 7 + 0] = position.xyz.x;
                    data[j * 7 + 1] = position.xyz.y;
                    data[j * 7 + 2] = position.xyz.z;
                    data[j * 7 + 3] = orientation.wxyz.w;
                    data[j * 7 + 4] = orientation.wxyz.x;
                    data[j * 7 + 5] = orientation.wxyz.y;
                    data[j * 7 + 6] = orientation.wxyz.z;
                }

                // Push data to LSL
                lsl_push_sample_f(outlet, data);
                k4abt_frame_release(body_frame); // Release body frame after use
            }
        }
    } while (++frame_count < 100); // Limit to 100 frames for this example

    // Cleanup and shutdown
    printf("Body tracking completed.\n");
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}