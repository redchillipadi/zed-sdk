///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/************************************************************
** This sample demonstrates how to read a SVO video file. **
** We use OpenCV to display the video.					   **
*************************************************************/

// ZED include
#include <sl/Camera.hpp>

// Sample includes
#include <opencv2/opencv.hpp>
#include "utils.hpp"

// Using namespace
using namespace sl;
using namespace std;

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");

int main(int argc, char **argv) {

    if (argc<=1)  {
        cout << "Usage: \n";
        cout << "$ ZED_SVO_Playback <SVO_file> \n";
        cout << "  ** SVO file is mandatory in the application ** \n\n";
        return EXIT_FAILURE;
    }

    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.input.setFromSVOFile(argv[1]);
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    PositionalTrackingParameters positional_tracking_parameters;
    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Positional Tracking", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    ObjectDetectionParameters detection_parameters;
    detection_parameters.image_sync = true;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_segmentation = true;

    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Object Detection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    auto resolution = zed.getCameraInformation().camera_configuration.resolution;
    // Define OpenCV window size (resize to max 720/404)
    sl::Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
    Mat svo_image(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
    cv::Mat svo_image_ocv = slMat2cvMat(svo_image);

    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    ObjectDetectionRuntimeParameters detection_parameters_rt;
    Objects objects;

    // Setup key, images, times
    char key = ' ';

    int svo_frame_rate = zed.getInitParameters().camera_fps;
    int nb_frames = zed.getSVONumberOfFrames();

    // Start SVO playback

     cout << "Timestamp (ms), Id, Label, Sublabel, State, Position x (m), Position y (m), Position z (m), Velocity x (m/s), Velocity y (m/s), Velocity z (m/s), Size x (m), Size y (m), Size z (m), Confidence" << endl;

     while (key != 'q') {
        returned_state = zed.grab();
        if (returned_state <= ERROR_CODE::SUCCESS) {
            // Get the side by side image
            zed.retrieveImage(svo_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);
            zed.getPosition(cam_pose, REFERENCE_FRAME::WORLD);

            int svo_position = zed.getSVOPosition();
            auto timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getMicroseconds();
            cv::putText(svo_image_ocv, to_string(timestamp), cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0));

            // Display the frame
            cv::imshow("View", svo_image_ocv);
            key = cv::waitKey(10);

            zed.retrieveObjects(objects, detection_parameters_rt);
            if (objects.is_new) {
                for (int i = 0; i < objects.object_list.size(); i++) {
                    cout << objects.timestamp << ", " << objects.object_list[i].id << ", " << objects.object_list[i].label << ", " << objects.object_list[i].sublabel << ", " << objects.object_list[i].action_state << ", ";
                    cout << objects.object_list[i].position[0]/1000.0 << ", " << objects.object_list[i].position[1]/1000.0 << ", " << objects.object_list[i].position[2]/1000.0 << ", ";
                    cout << objects.object_list[i].velocity[0]/1000.0 << ", " << objects.object_list[i].velocity[1]/1000.0 << ", " << objects.object_list[i].velocity[2]/1000.0 << ", ";
                    cout << objects.object_list[i].dimensions[0]/1000.0 << ", " << objects.object_list[i].dimensions[1]/1000.0 << ", " << objects.object_list[i].dimensions[2]/1000.0 << ", " << objects.object_list[i].confidence << endl;
                }
            }

            // svo_image.write(("capture_" + to_string(svo_position) + "-" + to_string(timestamp) + ".png").c_str());

            //ProgressBar((float)(svo_position / (float)nb_frames), 30);
        }
        else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            // Finish as end of file
            break;
        }
        else {
            print("Grab ZED : ", returned_state);
            break;
        }
     }

    // Disable modules
    zed.disableObjectDetection();
    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout <<"[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout<<" ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}
