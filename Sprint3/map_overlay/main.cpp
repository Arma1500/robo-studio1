#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
 
using namespace cv;
 
int main(int argc, char** argv )
{
    //SCALE FACTOR - to change based on map
    double scale_factor = 0.84;




    if ( argc != 3 ){
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    // ################# Get the map images
    Mat world_Map = imread(argv[1], IMREAD_COLOR);
    Mat slam_Map = imread(argv[2], IMREAD_GRAYSCALE);
    
    if (!world_Map.data || !slam_Map.data){
        printf("No image data \n");
        return -1;
    }

    // #################### Fix size of the smaller image
    //std::cout<< "Image Size: " << world_Map.size()<<std::endl;
    //std::cout<< "Image Channels: " << world_Map.channels()<<std::endl;
    
    // get size of the world map
    int xmin = 1000;
    int ymin = 1000;
    int xmax = 0;
    int ymax = 0;

    Mat world_gray = imread(argv[1], IMREAD_GRAYSCALE);
    int rows = world_gray.rows;
    int cols = world_gray.cols;

    for(int x = 0; x < rows; x++){
        for(int y = 0; y < cols; y++){
            
            // check every black pixel
            if(world_gray.at<uchar>(x,y) == 0){

                // get minimum point
                if(x < xmin){
                    xmin = x;
                }
                if(y < ymin){
                    ymin = y;
                }

                // get maximum point
                if(x > xmax){
                    xmax = x;
                }
                if(y > ymax){
                    ymax = y;
                }
            }
        }
    }

    int new_x = xmax - xmin;
    int new_y = ymax - ymin;


    cv::Mat new_slam;
    cv::resize(slam_Map, new_slam, cv::Size(new_y, new_x), 0, 0, cv::INTER_AREA);

    // scale to be inside the boundries
    cv::Mat scaled_slam;
    cv::resize(new_slam, scaled_slam, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);

    // ############## Overlay the Images 

    Mat overlayed = world_Map.clone(); // final new image
    
    //Colour World image - midpoint
    int w_r = overlayed.rows;
    int w_c = overlayed.cols;

    int w_midR = (int) w_r/2;
    int w_midC = (int) w_c/2;

    //Grayscale scaled image - midpoint
    int s_r = scaled_slam.rows;
    int s_c = scaled_slam.cols;

    int s_midR = (int) s_r/2;
    int s_midC = (int) s_c/2;

    //Get start and end points
    int start_r = w_midR - s_midR;
    int start_c = w_midC - s_midC;

    int end_r = w_midR + s_midR;
    int end_c = w_midC + s_midC;

    //Loop through and change the pixle values based on where the scaled slam image starts and ends
    for(int x = 0; x < w_r; x++){
        for(int y = 0; y < w_c; y++){
            
            if((x >= start_r && x <= end_r) && (y >= start_c && y <= end_c)){
                
                overlayed.at<Vec3b>(x,y)[0] = (0.25 * world_Map.at<Vec3b>(x,y)[0]) + 
                (0.75 * scaled_slam.at<u_char>(x-start_r,y-start_c)); //Alter the blue layer to the scaled slam map 
    
            } 

            //std::cout<<"x: " << x <<std::endl;

            if(x > 900){
                std::cout<<"is reaching the last x"<<std::endl;
            }
            //if(y > 900){
            //    std::cout<<"is reaching the last y"<<std::endl;
            //}
        }
    }

    // ############# Display
    //imshow("World Map", world_Map);
    //imshow("SLAM Map", slam_Map);
    //imshow("New Map", new_slam);
    //imshow("Scaled Map", scaled_slam);

    imshow("Overlayed Map", overlayed);

    waitKey(0);

 
    return 0;
}