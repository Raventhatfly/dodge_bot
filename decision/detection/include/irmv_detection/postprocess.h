/*
 * Disclaimer: This NMS implementation is based on existing open-source works.
 * Original implementations can be found in:
 * - https://github.com/emptysoal/TensorRT-YOLO11/
 * 
 * This version adds CUDA acceleration and pose handling modifications.
 */

#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <string>
#include <vector>

const int kGpuId = 0;
const int kNumClass = 1;
const int kNumKpt = 17;  // 单个目标对应的关键点的个数
const int kKptDims = 3;  // 单个关键点的维度，2 for x,y or 3 for x,y,visible
const int kInputH = 640;
const int kInputW = 640;
const float kNmsThresh = 0.45f;
const float kConfThresh = 0.25f;
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
const int kNumBoxElement = 7 + kNumKpt * kKptDims;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS), 51 keypoints

// const std::string onnxFile = "../onnx_model/yolo11s-pose.onnx";
// const std::string trtFile = "./yolo11s.plan";
// const std::string testDataDir = "../images";  // 用于推理

// for FP16 mode
const bool bFP16Mode = false;
// for INT8 mode
const bool bINT8Mode = false;
const std::string cacheFile = "./int8.cache";
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

const std::vector<std::string> vClassNames {"person"};

const std::vector<std::vector<int>> skeleton {
    {16, 14},
    {14, 12},
    {17, 15},
    {15, 13},
    {12, 13},
    {6, 12},
    {7, 13},
    {6, 7},
    {6, 8},
    {7, 9},
    {8, 10},
    {9, 11},
    {2, 3},
    {1, 2},
    {1, 3},
    {2, 4},
    {3, 5},
    {4, 6},
    {5, 7}
};


void transpose(float* src, float* dst, int numBboxes, int numElements, cudaStream_t stream);
/*
    transpose [56 8400] convert to [8400 56]
src:          Tensor, dim is [56 8400]
dst:          Tensor, dim is [8400 56]
numBboxes:    number of bboxes: default 8400
numElements:  center_x, center_y, width, height, 1 classes, 51 key points
*/

void decode(float* src, float* dst, int numBboxes, int numClasses, int numKpts, float confThresh, int maxObjects, int numBoxElement, cudaStream_t stream);
/*
    convert [8400 56] to [58001, ], 58001 = 1 + 1000 * (4bbox + cond + cls + keepflag + 51kpts), 1: number of valid bboxes
     1000: max bboxes, valid bboxes may less than 1000, 4bbox: left, top, right, bottom)
*/

void nms(float* data, float kNmsThresh, int maxObjects, int numBoxElement, cudaStream_t stream);


__inline__ void scale_bbox(cv::Mat& img, float bbox[4]){
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    float r = std::min(r_w, r_h);
    float pad_h = (kInputH - r * img.rows) / 2;
    float pad_w = (kInputW - r * img.cols) / 2;

    bbox[0] = (bbox[0] - pad_w) / r;
    bbox[1] = (bbox[1] - pad_h) / r;
    bbox[2] = (bbox[2] - pad_w) / r;
    bbox[3] = (bbox[3] - pad_h) / r;
}


__inline__ std::vector<std::vector<float>> scale_kpt_coords(cv::Mat& img, float* pkpt){
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    float r = std::min(r_w, r_h);
    float pad_h = (kInputH - r * img.rows) / 2;
    float pad_w = (kInputW - r * img.cols) / 2;

    std::vector<std::vector<float>> vScaledKpts;
    float x;
    float y;
    float conf;
    float* pSingleKpt;
    for (int i = 0; i < kNumKpt; i++){
        pSingleKpt = pkpt + i * kKptDims;
        x = pSingleKpt[0] - pad_w;
        y = pSingleKpt[1] - pad_h;
        x = x / r;
        y = y / r;
        conf = pSingleKpt[2];
        std::vector<float> scaledKpt {x, y, conf};
        vScaledKpts.push_back(scaledKpt);
    }

    return vScaledKpts;
}


#endif  // POSTPROCESS_H
