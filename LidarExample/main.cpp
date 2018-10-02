#include <flexiport/flexiport.h>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <typeinfo>
#include <cmath>

#include "matplotlibcpp.h"

//TODO: Find bug in code - can't accurately detect objects, maybe in logic of angle computation - flags are confirmed to work
//TODO: Clean up code + document structure

//NOTE: Current progress: structure of saving attributes to detected features works. Can currently detect start and end point, and whether cluster is a hole or object. Need to debug to fix core function, but data is there

// Use matplotlib C++ wrapper for python
namespace plt = matplotlibcpp;

class pointCluster
{
public:

    // Data Members
    int type_id;
    int start_pt;
    int end_pt;

    /*
    // Member Functions
    int computeWidth()
    {

    }

    std::vector<int> computeCenter()
    {

    }

    float computeAngle()
    {

    }

    float computeDistance()
    {

    }
     */
    void print()
    {
        std::cout << "Type ID:" << std::endl;
        std::cout << type_id << std::endl;
        std::cout << "Start Point:" << std::endl;
        std::cout << start_pt << std::endl;
        std::cout << "End Point:" << std::endl;
        std::cout << end_pt << std::endl;
    }
};

// Global Datasets
// Prepare data.
int temp_max = 682;// same value as num_pts, make sure to change once a better solution is found
std::vector<double> x(temp_max), y(temp_max);

// Function Declarations
int checkPoint(int index, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata);
std::vector<pointCluster> clusterPoints(int max_pts, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata);
void plotCluster(pointCluster cluster);

int main(int argc, char **argv)
{
    std::string port_options("type=serial,device=/dev/ttyACM0,timeout=1");
    double start_angle(0.0), end_angle(0.0);
    int first_step(-1), last_step(-1);
    int multiecho_mode(0);
    unsigned int speed(0), cluster_count(1);
    bool get_intensities(false), get_new(false), verbose(false);

    int test = 1;


#if defined(WIN32)
    port_options = "type=serial,device=COM4,timeout=1";
#endif // defined(WIN32)

    try
    {
        hokuyoaist::Sensor laser; // Laser scanner object
        // Set the laser to verbose mode (so we see more information in the
        // console)

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

        // Get some laser info
        std::cout << "Laser sensor information:\n";
        hokuyoaist::SensorInfo info;
        laser.get_sensor_info(info);
        std::cout << info.as_string();

        // Get range data
        hokuyoaist::ScanData data;
        laser.get_ranges(data, -1, -1, cluster_count);

        std::cout << "Measured data:\n";
        laser.get_ranges(data, -1, -1, cluster_count);
        //std::cout << data[400]; //how to get datapoints
        //std::cout << sizeof(data) << std::endl; //total datapoints in data
        int num_pts = 682; // In documentation - "Scanable steps: 682"
        int midpoint = num_pts/2;
        std::cout << num_pts << std::endl; //total datapoints in data
        // Save range data in laserdata
        int wall_start = 200;// Hardcoded wall points, 341 is midpoint
        int wall_end = 500;
        const float res = 0.00613592; // Resolution in radians/step
        float angle;
        int range;


        std::vector<double> rangedata(num_pts), angledata(num_pts);

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

            std::cout << std::to_string(angledata[i]*180/3.14159) + " is:";
            std::cout << rangedata.at(i) << std::endl;
        }

        int max_pts = num_pts;
        int threshold = 100; // Hardcoded test right now, change later
        int wall_distance = rangedata[midpoint]; // uses range data from midpoint as a guess for wall distance
        std::vector<pointCluster> clusters = clusterPoints(max_pts, threshold, wall_distance, rangedata, angledata); // Find clusters in dataset

        // Plot the data
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);
        // Plot line from given x and y data. Color is selected automatically.
        plt::plot(x, y, "b*");
        pointCluster c;
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

int checkPoint(int index, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata)
{
    int id = 0; // ID system: flag 0 for wall, 1 for object
    int point_distance = abs(rangedata.at(index) * sin(3.14/2 - angledata.at(index))); // Compute distance of point from wall
    if ((point_distance - wall_distance) > threshold) // Detect if the distance of the point from the wall is greater than a certain amount
    {
        id = 1; // Flag as object
    }
    return id;
}


std::vector<pointCluster> clusterPoints(int max_pts, int threshold, int wall_distance, std::vector<double> rangedata, std::vector<double> angledata)
{ // Datastructure: vector containing pointClusters
    std::vector<pointCluster> all_clusters(100); // arbitrarily set max number of things to 100
    std::vector<int> new_cluster_points(2); //make an array
    pointCluster new_cluster;

    for (int i=1; i<max_pts; i++)
    {
        int id_1;
        int id_2;
        new_cluster_points.at(0) = i;
        bool same_id = true;
        while (same_id == true)
        {
            i++;
            if(i >= max_pts-1)
            {
                new_cluster_points.at(1) = (i-1);
                break;
            }
            id_1 = checkPoint(i, threshold, wall_distance, rangedata, angledata);
            std::cout << id_1 << std:: endl;
            id_2 = checkPoint(i-1, threshold, wall_distance, rangedata, angledata);
            if (id_1 != id_2)
            {
                new_cluster_points.at(1) = (i);
                same_id = false;
            }
        }

        new_cluster.start_pt = new_cluster_points.at(0);
        new_cluster.end_pt = new_cluster_points.at(1);

        if (id_2 == 1)
        { new_cluster.type_id = 1; }
        else
        { new_cluster.type_id = 0; }

        all_clusters.push_back(new_cluster); // Somehow need to save this
        new_cluster.print();
    }
    return all_clusters;
}

void plotCluster(pointCluster cluster)
{
    std::vector<int> cluster_x(800); // arbritrarliy set max number of points in cluster
    std::vector<int> cluster_y(800);

    for (int i=cluster.start_pt; i<=cluster.end_pt; i++)
    {
        cluster_x.push_back(::x.at(i));//using global
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





















//std::vector<int> findPointIds()
//return point_ids = vector of 0s and 1s


/* CLUSTER SHOULD SAVE INDEX
 *
 *
 *
 *



 *
 *
 *
 *
 *         int wall_distance = 5;//average of middle points;
        int min_obj_size = 4; //in points
        int threshold = 5; // distance from wall required to be considered an object TODO: Specify units
        int wall_data [num_pts] = {};
        int obj_data [num_pts] = {};
        for (int i=wall_start; i<wall_end; i++)
        {
            if (abs(y.at(i) - wall_distance) > threshold)
            {
                //add to wall data
                //if next point is not in wall data, then that's a cluster'
            }
            else
            {

            }
        }
 *
 *
 *

globals:
 x
 y
 largest_hole [hole_start, hole_end] (index only)


function checkPoint(index)
     abs(y.at(i) - wall_distance) > threshold
     return 0 for wall, return 1 for object

function findPointIds()
    return point_ids = vector of 0s and 1s

function createIndex()
     hole_index = vector containing positions of all 0s
     obj_index = vector containing positions of all 1s

     for (int i=wall_start; i<wall_end; i++)
     {
        if (point_ids.at(i) == 0)
            {
                hole_index.push_back(i); //push_back is equivalent to .append in python
            }
        else
            {
                obj_index.push_back(i);
            }
     }

function createClusters()
 create matrix with clusters as columns for each

function findLargestHole()
 iterate through each column and find the column with the largest hole
    largest hole [hole_start, hole_end]
    hole width = distance between hole_start and hole_end (use distance formula on x and y)
    return [hole_start, hole_end]

function findObjects()
 for each cluster, find average point (average x and y values)
    save each point in a matrix
    convert point to polar coordinates for angle and distance
    return average points

plot clusters in a particular color
plot holes in a particular color






 for indexing through vector
    if


function checkCluster()


 prev_id = 0; //Assume first is wall
 for (int i=wall_start; i<wall_end; i++)
    id = checkPoint(i)
    if prev_id == id

    return last index


function checkConsecutive(cluster_list)
    cluster_list.append(new_cluster)

for cluster in cluster_list


 */