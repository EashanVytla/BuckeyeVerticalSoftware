#include "opencv4/opencv2/opencv.hpp"
#include "tesseract/baseapi.h"
#include "leptonica/allheaders.h"

int main() {
    // Read the image
    cv::Mat img = cv::imread("../data/OCR5x2.png");

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Resize the image
    cv::resize(gray, gray, cv::Size(320, gray.rows * 320 / gray.cols));

    // Apply Gaussian blur
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    // Apply Otsu's thresholding
    cv::Mat th3;
    cv::threshold(gray, th3, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // Apply dilation to thicken the edges
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
    cv::erode(th3, th3, kernel, cv::Point(-1, -1), 1);
    cv::dilate(th3, th3, kernel_dilate, cv::Point(-1, -1), 1);

    // Set the rotation angle (in degrees)
    double rotation_angle = 0;

    // Calculate rotation matrix
    cv::Point2f center(static_cast<float>(th3.cols) / 2, static_cast<float>(th3.rows) / 2);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, rotation_angle, 1.0);

    // Apply rotation to the image
    cv::Mat rotated_image;
    cv::warpAffine(th3, th3, rotation_matrix, th3.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));

    char *outText;

    tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
    // Initialize tesseract-ocr with English, without specifying tessdata path
    if (api->Init(NULL, "eng")) {
        fprintf(stderr, "Could not initialize tesseract.\n");
        exit(1);
    }

    api->SetImage(th3.data, th3.cols, th3.rows, 1, th3.cols);

    api->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);

    api->SetImage(th3.data, th3.cols, th3.rows, 1, th3.cols);

    // Get OCR result
    outText = api->GetUTF8Text();
    printf("OCR output:\n%s", outText);

    cv::imshow("Window Name", th3);

    cv::waitKey(0); 

    // Destroy used object and release memory
    api->End();
    delete api;
    delete [] outText;

    return 0;
}