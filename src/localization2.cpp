#include "localization.h"

Mat projectPixel(double zConst, double u, double v,
                 cv::Mat &inverseCameraMatrix,
                 cv::Mat &inverseRotationMatrix,
                 cv::Mat &translationVector)
{

    // cout << u << ", " << v << endl;
    // cout << "INSIDE PROJECT" << endl;

    Mat pixelVector(3, 1, inverseCameraMatrix.type());

    pixelVector.at<double>(0, 0) = u;
    pixelVector.at<double>(1, 0) = v;
    pixelVector.at<double>(2, 0) = 1;

    // cout << "NOW PIXEL VEC: " << pixelVector << endl;
    // cout << endl << endl;

    // cout << "FIRST PART" << endl;
    // cout << inverseCameraMatrix * pixelVector << endl;


    Mat leftSide = inverseRotationMatrix * (inverseCameraMatrix * pixelVector);
    Mat rightSide = inverseRotationMatrix * translationVector;

    // cout << "LEFT" << endl << endl;
    // cout << leftSide << endl;

    // cout << "RIGHT" << endl << endl;
    // cout << rightSide << endl;

    // cout << endl;
    // cout << "PAST LEFT RIGHT SIDE" << endl;

    // cout << "for " << u << " " << v << endl;

    double scalingFactor = (zConst + rightSide.at<double>(2, 0)) / leftSide.at<double>(2, 0);

    // cout << "scaling factor : " << endl << scalingFactor << endl;

    Mat worldCoordinate = inverseRotationMatrix * (((scalingFactor * inverseCameraMatrix) * pixelVector) - translationVector);

    // cout << "FINAL WORLDCOORD: " << endl << worldCoordinate << endl;

    return worldCoordinate;
}

std::pair<double, double> offset_to_latlon(double lat, double lon, double heading, double offset_x, double offset_y) {
    // Earth's radius in meters
    const double R = 6371000;

    // Convert latitude and longitude to radians
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // Convert heading to radians and adjust it to align with the camera frame
    double heading_rad = (90.0 - heading) * M_PI / 180.0;

    //std::cout << "Heading radians: " << heading_rad << std::endl;

    // Rotate the offset coordinates based on the heading
    double rotated_offset_x = offset_x * std::cos(heading_rad) - offset_y * std::sin(heading_rad);
    double rotated_offset_y = offset_x * std::sin(heading_rad) + offset_y * std::cos(heading_rad);

    //std::cout << "rotated offset x: " << rotated_offset_x << std::endl;
    //std::cout << "rotated offset y: " << rotated_offset_y << std::endl;

    // Calculate the angular distance
    double angular_dist = std::sqrt(rotated_offset_x * rotated_offset_x + rotated_offset_y * rotated_offset_y) / R;

    //std::cout << "angular_dist: " << angular_dist << std::endl;

    // Calculate the bearing
    double bearing = std::atan2(rotated_offset_y, -rotated_offset_x);

    //std::cout << "bearing: " << bearing << std::endl;

    // Calculate the new latitude
    double new_lat_rad = std::asin(std::sin(lat_rad) * std::cos(angular_dist) +
                                   std::cos(lat_rad) * std::sin(angular_dist) * std::cos(bearing));

    //std::cout << "new_lat_rad: " << new_lat_rad << std::endl;

    // Calculate the new longitude
    double new_lon_rad = lon_rad + std::atan2(std::sin(bearing) * std::sin(angular_dist) * std::cos(lat_rad),
                                              std::cos(angular_dist) - std::sin(lat_rad) * std::sin(new_lat_rad));

    //std::cout << "new_lon_rad: " << new_lon_rad << std::endl;

    // Convert the new latitude and longitude back to degrees
    double new_lat = new_lat_rad * 180.0 / M_PI;
    double new_lon = new_lon_rad * 180.0 / M_PI;

    return std::make_pair(new_lat, new_lon);
}

Mat alignedProject(double zConst, double u, double v, double originX, double originY,
                   cv::Mat &inverseCameraMatrix,
                   cv::Mat &inverseRotationMatrix,
                   cv::Mat &translationVector)
{

    // cout << "INSIDE ALGIEND" << endl;

    Mat result(2, 1, inverseCameraMatrix.type());

    Mat projectedPoint = projectPixel(zConst, u, v, inverseCameraMatrix, inverseRotationMatrix, translationVector);
    Mat refX = projectPixel(zConst, originX, v, inverseCameraMatrix, inverseRotationMatrix, translationVector);
    Mat refY = projectPixel(zConst, u, originY, inverseCameraMatrix, inverseRotationMatrix, translationVector);

    // cout << "PAST FIRAST PROJECT" << endl;

    // cout << "REFX - projectPOint" << refX - projectedPoint << endl;
    // cout << "REFY - projectPOint" << refY - projectedPoint << endl;

    double distanceX = norm(refX - projectedPoint);
    double distanceY = norm(refY - projectedPoint);

    // cout << distanceX << endl;

    if (u < originX)
        distanceX *= -1;

    if (v > originY)
        distanceY *= -1;

    result.at<double>(0, 0) = distanceX;
    result.at<double>(1, 0) = distanceY;

    return result;

}
