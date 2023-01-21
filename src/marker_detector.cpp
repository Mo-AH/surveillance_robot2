/*! 
 * \file marker_detector.cpp
 * \brief Ros node to detect markers from the camera image and to retrieve the rooms info
 * \author Mohammad Al Horany
 * \version 1.0
 * \date 21/01/23
 * 
 * \param [in] total_markers Total number of markers to be found
 *
 * \details
 * 
 * Subscribes to: <BR>
 *   - /robot/camera/image_raw
 * 
 * Publishes to: <BR>
 *   - /map/rooms
 * 
 * Client of: <BR>
 *   - /room_info 
 * 
 * Description : 
 * Node to detect markers in the camera image.
 * It asks to the marker_server node the room's informations embedded
 * in the marker and send those informations to the smach_robot node.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <map>
#include <vector>
#include <ros/service_client.h>
#include <surveillance_robot2/Room.h>
#include <surveillance_robot2/RoomConnection.h>
#include <surveillance_robot2/RoomInformation.h>

/*!
 * \class ArucoMarkerDetector
 *
 * \brief A class for detecting markers in the camera images and for publishing informations about rooms.
 *
 * This class detects the Aruco markers in the camera image, asks for room informations embedded in them through
 * the `/room_info` service (provided by `marker_server`) and publish those informations in the `map/rooms` topic,
 * to which the `smach_robot` is subscribed.
 * 
 */

class ArucoMarkerDetector
{
    ros::NodeHandle nh_;                    //!< ROS Node handler

    image_transport::ImageTransport it_;    //!< Image transport to receive the camera image
    image_transport::Subscriber image_sub;  //!< ROS image subscriber
    aruco::MarkerDetector detector;         //!< Aruco library's marker detector object

    std::map<int, bool> markers_detected;   //!< Map to store if a marker has been detected or not
    std::vector<int> markers_ids;           //!< Vector to store the ids of the detected markers
    int total_markers;                      //!< Total number of markers that should be detected

    ros::Publisher room_info_pub;           //!< ROS Publisher for rooms information
    ros::ServiceClient marker_info_client;  //!< Service client to retrieve room information embedded in the marker

public:
    /**
     * @brief Constructor
     * Initializes the objects, subscribes to the image topic and gets the total number of markers
     */
    ArucoMarkerDetector()
        : it_(nh_)
    {
        // Publish the informations of the rooms
        room_info_pub = nh_.advertise<surveillance_robot2::Room>("/map/rooms",1);
        while (room_info_pub.getNumSubscribers() < 1)
            ros::Duration(0.2).sleep();

        // Subscribe to the image topic
        image_sub = it_.subscribe("/robot/camera/image_raw", 1, &ArucoMarkerDetector::imageCallback, this);
                
        // Get the total number of markers to be found
        nh_.getParam("map/total_markers", total_markers);

        // ServiceClient to retrieve room info embedded in the marker
        marker_info_client = nh_.serviceClient<surveillance_robot2::RoomInformation>("/room_info");
        marker_info_client.waitForExistence();
    }

    /**
     * @brief Destructor
     */
    ~ArucoMarkerDetector()
    {
    }

    /**
     * @brief Callback function for the `/robot/camera/image_raw` topic
     * @param msg Image message
     *
     * This function detects markers in the received image, draws them on the image, retrieves room information
     * embedded in the marker and send them to `/map/rooms` topic. When it found all the total number of markers,
     * it shutdowns the node.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Convert the ROS image message to a OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect markers in the image
        std::vector<aruco::Marker> markers;
        detector.detect(cv_ptr->image, markers);

        // Draw the markers on the image
        for (unsigned int i = 0; i < markers.size(); i++)
        {
            markers[i].draw(cv_ptr->image, cv::Scalar(0, 0, 255), 2);
            int marker_id = markers[i].id;

            // Marker not seen before
            if (!markers_detected[marker_id])
            {
                markers_detected[marker_id] = true;
                
                // Print marker id
                std::cout << "\n[@aruco_marker_detector] Detected new aruco marker - ID = ";          
                std::cout << marker_id << " ##" << std::endl;                                                       
                
                // Call service to retrieve marker's information
                surveillance_robot2::RoomInformation srv;
                srv.request.id = marker_id;

                if (marker_info_client.call(srv) and srv.response.x)
                {
                    // Informative marker found
                    markers_ids.push_back(marker_id);

                    // Print room informations of the marker
                    std::cout << "Room Name = " << srv.response.room.c_str() << "\nRoom coordinates = [ " ;
                    std::cout << srv.response.x << ", " << srv.response.y << " ]" << std::endl;

                    // Publish room informations
                    surveillance_robot2::Room room_msg;
                    room_msg.room = srv.response.room;
                    room_msg.x = srv.response.x;
                    room_msg.y = srv.response.y;
                    for (std::size_t j = 0; j < srv.response.connections.size(); j++){
                        room_msg.connections.push_back(srv.response.connections.at(j));
                    }
                    room_info_pub.publish(room_msg);
                }
                else
                {
                    ROS_WARN("Couldn't get room informations for this marker");
                }
            }

            // If all informative markers have been found, shutdown this node
            if(markers_ids.size() == total_markers)
            {
                // Publish an empty msg to comunicate the end of the building process
                surveillance_robot2::Room empty_msg;
                room_info_pub.publish(empty_msg);

                // Stop execution
                std::cout << "\n[@aruco_marker_detector] : All " << total_markers;
                std::cout << " markers found! This node will shutdown." << std::endl;
                ros::Duration(5).sleep();
                ros::shutdown();
            }
        }

        // Show the image with the markers
        cv::imshow("Aruco markers", cv_ptr->image);
        cv::waitKey(1);
    }
};


/**
 * @brief Main function of the Aruco Marker Detector node
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 *
 * This function initializes the ROS node, creates an instance of the ArucoMarkerDetector class,
 * and spins. When all markers are found, this node will shutdown.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_marker_detector");

    std::cout << "[@aruco_marker_detector] : Node started. I will run until all required markers are found!" << std::endl;

    ArucoMarkerDetector detector;
    ros::spin();
    return 0;
}