/*

Author: Mirhan Ürkmez
Date: 22/10/2020
Description: Codes for hand and human detection on RGB images using yolov4 models are included.    

*/


#include "detection.h"

using namespace std;
using namespace cv;
using namespace dnn;
using namespace std::chrono;

hand_human_detector::hand_human_detector(std::string cfg_file, std::string weights_file)
{
    this->cfg_file = cfg_file;
    this->weights_file = weights_file;
}

void hand_human_detector::getRects(cv::Mat& frame, const std::vector<cv::Mat>& outs, vector<Rect>& hands, Rect& human)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    static int save_num = 0;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > conf_threshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, conf_threshold, nms_thresh, indices);
    vector<Rect> NMSboxes;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        if (classIds[idx] == 0)
            hands.push_back(box);
        else if (classIds[idx] == 1)
            human = box;
    }
}

vector<String> hand_human_detector::getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

void hand_human_detector::run(Mat& frame, vector<Rect>& hands, Rect& human)
{
    hands.clear();
    static auto net = cv::dnn::readNetFromDarknet(cfg_file, weights_file);
    //net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    std::vector<cv::String> output_names;
    output_names = net.getUnconnectedOutLayersNames();
    cv::Mat blob;
    std::vector<cv::Mat> detections;
    cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(blob_width, blob_height), cv::Scalar(), true, false, CV_32F);
    net.setInput(blob);

    net.forward(detections, output_names);

    getRects(frame, detections, hands, human);

}
