/*
 * Collects data from a Lidar scan, groups data points and identifies them as walls or objects,
 * then displays data in a nice visual
 *
 * Last edited: 10/5/18
 * Author: Amy Phung
 *
 * Requires:
 *  flexiport (dependency of hokuyoaist)
 *  hokuyoaist (library for getting data from lidar)
 *  matplotlib (plotting library)
 *
 * Notes:
 *  Currently relies on midpoint to determine wall distance and assumes lidar is perpendicular
 *  to the wall
 *  Since matplotlib library is usally a python library and we're using a C++ wrapper for it,
 *  real-time plotting doesn't work. If REAL_TIME_VISUAL is set to false, a single frame of the
 *  color coded plot will be displayed. If this is set to true, the data will be printed in the
 *  command line real-time but with no formatting or color coding
 */

#include <iostream>
#include <flexiport/flexiport.h>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include "matplotlibcpp.h"

bool REAL_TIME_VISUAL = false; // set to true for no-color, real-time display

//TODO: Make code not reliant on perpendicular wall
//TODO: Handle the case when a person/object is in the middle of a scan
//TODO: Have an object threshold size to avoid picking up noise as objects
//TODO: Make pretty real-time visualization
//TODO: Use webcam input to determine color of objects
//TODO: Fill in code for pointCluster member functions
//TODO: Determine units of distance measurement
//TODO: Remove unneccessary hardcode

//Current progress:
// Structure of saving attributes to detected features works.
// Can currently detect start and end point, and whether cluster is a hole or object.

//CODE START************************************************************//
// Use matplotlib C++ wrapper for python
namespace plt = matplotlibcpp;

//pointCluster CLASS****************************************************//
class pointCluster // For each cluster of points, a pointCluster object is
                   // created to save attributes of the cluster
{
public: // Public data members can be accessed anywhere in code
    // Data Members
    int type_id; // 1 for object, 0 for wall
    int start_pt; // Index of start point
    int end_pt; // Index of end point

    /*
    // Member Functions
    int computeWidth()
    {
        // Fill in code here to determine object/hole width in meters given start pt and end pt
        // Keep in mind that a constant interval of index does not necessarily mean a constant width in meters
    }

    std::vector<int> computeCenter()
    {
        // Compute the "center of mass" of the cluster by averaging x and y values
    }

    float computeAngle()
    {
        // Compute the angle of the "center of mass" of the cluster from the lidar - assume straight is 0,
        // left is positive, and right is negative (right hand rule conventions)
    }

    float computeDistance()
    {
        // Compute the distance to the "center of mass" from the lidar
    }
     */
    void print() // Print data for verification
    {
        std::cout << "Type ID:" << std::endl;
        std::cout << type_id << std::endl;
        std::cout << "Start Point:" << std::endl;
        std::cout << start_pt << std::endl;
        std::cout << "End Point:" << std::endl;
        std::cout << end_pt << std::endl;
    }
};

//GLOBAL DATASETS*******************************************************//
// Prepare data.
std::vector<double> x(682), y(682); // 682 is the number of points the lidar scans. This was a workaround
                                    // since I couldn't figure out how to give the callback function in for_each
                                    // parameters, so I had to make this a global variable - eventally, it'd be a good
                                    // idea to make this not global

//FUNCTION PROTOTYPES***************************************************//
int checkPoint(int index, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata);
std::vector<pointCluster> clusterPoints(int max_pts, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata);
void plotCluster(pointCluster cluster);


//MAIN FUNCTION********************************************************//
int main(int argc, char **argv)
{
    // Variables
    std::string port_options("type=serial,device=/dev/ttyACM0,timeout=1");
    int multiecho_mode(0);
    unsigned int speed(0), cluster_count(1);
    int num_pts = 682; // In lidar documentation - "Scanable steps: 682"
    int midpoint = num_pts/2; // Midpoint is in middle of scan
    int wall_start = 200; // Hardcoded wall points, 341 is midpoint
    int wall_end = 500;
    int threshold = 200; // Hardcoded difference between midpoint and object to warrant detection
    const float res = 0.00613592; // Resolution in radians/step - from lidar documentation
    int wall_distance;
    float angle;
    int range;
    hokuyoaist::ScanData data; // For saving raw lidar data
    std::vector<pointCluster> clusters; // For saving vector containing all pointCluster objects
    std::vector<double> rangedata(num_pts), angledata(num_pts); // For saving the range and angle data in an easy-to-manipulate vector

#if defined(WIN32)
    port_options = "type=serial,device=COM4,timeout=1";
#endif // defined(WIN32)

    try
    {
        hokuyoaist::Sensor laser; // Create new laser scanner object

        // Open the laser
        laser.open(port_options);

        // Calibrate the laser time stamp
        std::cout << "Calibrating laser time\n";
        laser.calibrate_time();
        std::cout << "Calculated offset: " << laser.time_offset() << "ns\n";
        std::cout << "Calculated drift rate: " << laser.drift_rate() << '\n';
        std::cout << "Calculated skew alpha: " << laser.skew_alpha() << '\n';

        // Turn the laser on
        laser.set_power(true);

        // I'm not sure what this block of code does but it seems important *******************************
        // Set the motor speed
        try
        {
            laser.set_motor_speed(speed);
        }
        catch(hokuyoaist::MotorSpeedError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        catch(hokuyoaist::ResponseError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        // Set multiecho mode
        switch(multiecho_mode)
        {
            case 1:
                laser.set_multiecho_mode(hokuyoaist::ME_FRONT);
                break;
            case 2:
                laser.set_multiecho_mode(hokuyoaist::ME_MIDDLE);
                break;
            case 3:
                laser.set_multiecho_mode(hokuyoaist::ME_REAR);
                break;
            case 4:
                laser.set_multiecho_mode(hokuyoaist::ME_AVERAGE);
                break;
            case 0:
            default:
                laser.set_multiecho_mode(hokuyoaist::ME_OFF);
                break;
        }

        //**********************************************************************************************

        // Get some laser info
        std::cout << "Laser sensor information:\n";
        hokuyoaist::SensorInfo info;
        laser.get_sensor_info(info);
        std::cout << info.as_string();

        while(true)
        {
            laser.get_ranges(data, -1, -1, cluster_count);

            for (int i=wall_start; i<wall_end; i++)
            {
                // Create laserdata dataset
                range = data[i];
                rangedata.at(i) = range;

                // Create angledata dataset
                angle = (i-midpoint)*res;
                angledata.at(i) = angle;

                // Save cartesian coordinates to plotting vectors
                x.at(i) = range*sin(angle);
                y.at(i) = range*cos(angle);
            }

            wall_distance = rangedata[midpoint]; // uses range data from midpoint as a guess for wall distance
            clusters = clusterPoints(num_pts, threshold, wall_distance, rangedata, angledata); // Find clusters in dataset

            std::cout << std::endl;

            if (REAL_TIME_VISUAL == false) // Code will only run once to gather data if REAL_TIME_VISUAL is set to false
            {
                break;
            }
        }

        // Plot the data
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);
        // Plot line from given x and y data. Color is selected automatically.
        plt::plot(x, y, "b*");
        for_each(clusters.begin(), clusters.end(), plotCluster);
        // Set x-axis to interval [0,1000000]
        plt::xlim(0, 1000*1000);
        // Enable legend.
        plt::legend();
        plt::axis("equal");
        // Save the image (file format is determined by the extension)
        plt::show();

        // Close the laser
        laser.close();
    }
    catch(hokuyoaist::BaseError &e)
    {
        std::cerr << "Caught exception: " << e.what() << '\n';
        return 1;
    }
    return 0;
}

//FUNCTIONS*************************************************************//
int checkPoint(int index, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata)
{
    int id = 0; // ID system: flag 0 for wall, 1 for object
    int point_distance = abs(rangedata.at(index) * sin(3.14/2 - angledata.at(index))); // Compute distance of point from wall
    if (abs(point_distance - wall_distance) > threshold) // Detect if the distance of the point from the wall is greater than a certain amount
    {
        id = 1; // Flag as object
    }
    return id;
}

std::vector<pointCluster> clusterPoints(int max_pts, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata)
{ // Datastructure: vector containing pointClusters
    std::vector<pointCluster> all_clusters(100); // arbitrarily set max number of things to detect to 100
    std::vector<int> new_cluster_points(2); //make an array to save start and end points
    pointCluster new_cluster; // make new pointCluster object

    for (int i=1; i<max_pts; i++) // Checks if the adjacent points have the same id - object points will have an id of 1, wall points will have an id of 0
    {
        int id_1;
        int id_2;
        new_cluster_points.at(0) = i; // Start point of cluster is the first index
        bool same_id = true;
        while (same_id == true) // While adjacent points have the same id
        {
            i++; // increment i
            if(i >= max_pts-1) // if i goes past last data point, break out of loop
            {
                new_cluster_points.at(1) = (i-1);
                break;
            }
            id_1 = checkPoint(i, threshold, wall_distance, rangedata, angledata); // check id of first point
            std::cout << id_1;
            id_2 = checkPoint(i-1, threshold, wall_distance, rangedata, angledata); // check id of adjacent point
            if (id_1 != id_2) // if the two points aren't both walls or objects
            {
                new_cluster_points.at(1) = (i); // log the last point as the current index
                same_id = false; // break out of while loop
            }
        }

        new_cluster.start_pt = new_cluster_points.at(0); // set start point of cluster equal to first saved point
        new_cluster.end_pt = new_cluster_points.at(1); // set last point of cluster equal to second saved point

        if (id_2 == 1)
        { new_cluster.type_id = 1; } // assign id to cluster
        else
        { new_cluster.type_id = 0; } // assign id to cluster

        all_clusters.push_back(new_cluster); // Add new cluster to all_clusters vector
    }
    return all_clusters; // return vector containing all the found clusters
}

void plotCluster(pointCluster cluster)
{
    std::vector<int> cluster_x(800); // arbritrarliy set max number of points in cluster
    std::vector<int> cluster_y(800);

    for (int i=cluster.start_pt; i<=cluster.end_pt; i++)
    {
        cluster_x.push_back(::x.at(i));//using global datatset
        cluster_y.push_back(::y.at(i));
    }
    if (cluster.type_id == 0) // Plot wall as red
    {
        plt::plot(cluster_x, cluster_y, "r*");
    }
    else // Plot objects as green
    {
        plt::plot(cluster_x, cluster_y, "g*");
    }
}