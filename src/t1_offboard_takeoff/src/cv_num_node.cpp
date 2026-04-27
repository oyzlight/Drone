/**
 * @brief 25年电赛H题，完整巡航，通信，识别
 * @author 23届seeker战队，小怪，雷总，源神
 * @date 2025.8.24
 */


#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <t1_offboard_takeoff/dxy.h>
#include <t1_offboard_takeoff/wild.h>
#include <t1_offboard_takeoff/Point.h>
#include <t1_offboard_takeoff/clean_wild.h>
#include <t1_offboard_takeoff/stop.h>
#include <t1_offboard_takeoff/Detect_animals.h>
#include <map>
#include <set>

// Color definitions
const cv::Scalar BOX_COLOR = cv::Scalar(255, 0, 0);         // Detection box: blue
const cv::Scalar ROI_COLOR = cv::Scalar(0, 0, 255);         // ROI box: red
const cv::Scalar IMG_CENTER_COLOR = cv::Scalar(0, 255, 255);  // Image center: yellow
const cv::Scalar TARGET_CENTER_COLOR = cv::Scalar(0, 0, 255); // Target center: red
const cv::Scalar ARROW_COLOR = cv::Scalar(255, 0, 255);       // Arrow: magenta
const cv::Scalar COUNT_COLOR = cv::Scalar(0, 255, 0);         // Count text: green

t1_offboard_takeoff::dxy dxy;
uint8_t is_clean_wild=0;
t1_offboard_takeoff::stop stop_clean;
t1_offboard_takeoff::Detect_animals detect_animals;


// Class colors (label background)
const std::vector<cv::Scalar> CLASS_COLORS = {
    cv::Scalar(0, 0, 255),    // Class 0 (E): red
    cv::Scalar(0, 255, 0),    // Class 1 (K): green
    cv::Scalar(0, 255, 255),  // Class 2 (M): yellow
    cv::Scalar(255, 0, 255),  // Class 3 (T): magenta
    cv::Scalar(255, 255, 0)   // Class 4 (W): cyan
};

// Class names
const std::vector<std::string> class_names = {"E", "K", "M", "T", "W"};

// Draw bounding box and label
void plotOneBox(const cv::Rect& box, cv::Mat& img, int class_id, const std::string& label,
               int lineThickness = -1) {
    int tl = (lineThickness == -1) ? 
             round(0.002 * (img.rows + img.cols) / 2) + 1 : lineThickness;
    int tf = std::max(tl - 1, 1); // 文字线条厚度

    // 1. 计算文字实际尺寸（严格基于精简后的label）
    int baseLine = 0;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, tl / 3.0, tf, &baseLine); 
    // 注意：字体缩放因子从 tl/2.0 改为 tl/3.0，避免文字过大导致矩形撑长

    // 2. 调整标签原点，避免超出图像边界
    cv::Point labelOrigin(box.x, std::max(box.y - labelSize.height - 3, 3)); // 上边界限制

    // 3. 背景矩形尺寸：严格匹配文字尺寸（仅添加必要边距）
    cv::Rect bg_rect(
        labelOrigin.x, 
        labelOrigin.y, 
        labelSize.width,                  // 宽度=文字宽度（无多余延伸）
        labelSize.height + baseLine + 2   // 高度=文字高度+基线+2像素边距（减少冗余）
    );

    // 绘制背景和文字
    cv::rectangle(img, bg_rect, CLASS_COLORS[class_id], -1, cv::LINE_AA); // 背景
    cv::putText(img, label, 
               cv::Point(labelOrigin.x, labelOrigin.y + labelSize.height + tf - 1), // 文字位置
               cv::FONT_HERSHEY_SIMPLEX, tl / 3.0, cv::Scalar(0, 0, 0), tf, cv::LINE_AA);
}


// Generate grid
cv::Mat makeGrid(int w, int h) {
    cv::Mat grid(1, h * w, CV_32FC2);
    cv::Vec2f* ptr = grid.ptr<cv::Vec2f>();
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            *ptr++ = cv::Vec2f(x, y);
        }
    }
    return grid.reshape(1, h * w);
}

// Calculate output coordinates
void calOutputs(cv::Mat& outs, int nl, int na, int modelW, int modelH,
               const std::vector<cv::Mat>& anchorGrid, const std::vector<float>& stride) {
    std::vector<std::pair<int, int>> featureMapSizes = {{40, 40}, {20, 20}, {10, 10}};
    int rowInd = 0;
    
    for (int i = 0; i < nl; ++i) {
        int h = featureMapSizes[i].first;
        int w = featureMapSizes[i].second;
        int numBoxesPerScale = na * h * w;
        
        cv::Mat scaleOutput = outs.rowRange(rowInd, rowInd + numBoxesPerScale);
        cv::Mat grid = makeGrid(w, h);
        cv::Mat gridRepeat = cv::repeat(grid, na, 1);
        
        for (int r = 0; r < scaleOutput.rows; ++r) {
            float* data = scaleOutput.ptr<float>(r);
            data[0] = (data[0] * 2.0f - 0.5f + gridRepeat.at<float>(r, 0)) * stride[i];
            data[1] = (data[1] * 2.0f - 0.5f + gridRepeat.at<float>(r, 1)) * stride[i];
        }
        
        for (int a = 0; a < na; ++a) {
            int startIdx = a * h * w;
            int endIdx = startIdx + h * w;
            float anchorW = anchorGrid[i].at<float>(a, 0);
            float anchorH = anchorGrid[i].at<float>(a, 1);
            
            for (int r = startIdx; r < endIdx; ++r) {
                float* data = scaleOutput.ptr<float>(r);
                data[2] = (data[2] * 2.0f) * (data[2] * 2.0f) * anchorW;
                data[3] = (data[3] * 2.0f) * (data[3] * 2.0f) * anchorH;
            }
        }
        
        rowInd += numBoxesPerScale;
    }
}

// Post-processing: NMS and coordinate conversion
void postProcess(cv::Mat& outs, int modelH, int modelW, int imgH, int imgW,
                float nmsThresh, float confThresh,
                std::vector<cv::Rect>& boxes, std::vector<float>& confs, std::vector<int>& ids) {
    boxes.clear();
    confs.clear();
    ids.clear();
    
    std::vector<int> indices;
    for (int i = 0; i < outs.rows; ++i) {
        float conf = outs.at<float>(i, 4);
        if (conf > confThresh) {
            indices.push_back(i);
        }
    }
    if (indices.empty()) return;
    
    std::vector<cv::Rect> tempBoxes;
    std::vector<float> tempConfs;
    std::vector<int> tempIds;
    
    for (int i : indices) {
        float cx = outs.at<float>(i, 0) / modelW * imgW;
        float cy = outs.at<float>(i, 1) / modelH * imgH;
        float w = outs.at<float>(i, 2) / modelW * imgW;
        float h = outs.at<float>(i, 3) / modelH * imgH;
        
        int x1 = std::max(0, (int)(cx - w / 2));
        int y1 = std::max(0, (int)(cy - h / 2));
        int x2 = std::min(imgW - 1, (int)(cx + w / 2));
        int y2 = std::min(imgH - 1, (int)(cy + h / 2));
        

        int box_width = x2 - x1;
        int box_height = y2 - y1;
        if (box_width < 10 || box_height < 10 || box_width > 320 || box_height > 320) {
            continue;
        }
        tempBoxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
        tempConfs.push_back(outs.at<float>(i, 4));
        
        cv::Mat clsScores = outs.row(i).colRange(5, outs.cols);
        double minVal, maxVal;
        cv::Point maxLoc;
        cv::minMaxLoc(clsScores, &minVal, &maxVal, nullptr, &maxLoc);
        tempIds.push_back(maxLoc.x);
    }
    
    std::vector<int> nmsIndices;
    cv::dnn::NMSBoxes(tempBoxes, tempConfs, confThresh, nmsThresh, nmsIndices);
    
    for (int i : nmsIndices) {
        boxes.push_back(tempBoxes[i]);
        confs.push_back(tempConfs[i]);
        ids.push_back(tempIds[i]);
    }
}

// Inference function
void inferImg(const cv::Mat& img0, Ort::Session& session, int modelH, int modelW,
             int nl, int na, const std::vector<float>& stride, 
             const std::vector<cv::Mat>& anchorGrid,
             std::vector<cv::Rect>& boxes, std::vector<float>& confs, std::vector<int>& ids,
             float nmsThresh = 0.2f, float confThresh = 0.4f) {
    cv::Mat img;
    cv::resize(img0, img, cv::Size(modelW, modelH), 0, 0, cv::INTER_AREA);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    img.convertTo(img, CV_32F, 1.0 / 255.0);
    
    cv::Mat inputBlob;
    cv::dnn::blobFromImage(img, inputBlob, 1.0, cv::Size(modelW, modelH), 
                          cv::Scalar(), false, false);
    
    std::vector<int64_t> inputShape = {1, 3, modelH, modelW};
    auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo,
        inputBlob.ptr<float>(),
        inputBlob.total(),
        inputShape.data(),
        inputShape.size()
    );
    
    Ort::AllocatorWithDefaultOptions allocator;
    auto inputName = session.GetInputNameAllocated(0, allocator);
    auto outputName = session.GetOutputNameAllocated(0, allocator);
    std::vector<const char*> inputNames = {inputName.get()};
    std::vector<const char*> outputNames = {outputName.get()};
    
    auto outputs = session.Run(Ort::RunOptions{nullptr},
                              inputNames.data(), &inputTensor, 1,
                              outputNames.data(), outputNames.size());
    
    Ort::TensorTypeAndShapeInfo outputInfo = outputs[0].GetTensorTypeAndShapeInfo();
    std::vector<int64_t> outputShape = outputInfo.GetShape();
    int outRows = outputShape[1];
    int outCols = outputShape[2];
    float* outputData = outputs[0].GetTensorMutableData<float>();
    cv::Mat outs(outRows, outCols, CV_32F, outputData);
    
    calOutputs(outs, nl, na, modelW, modelH, anchorGrid, stride);
    postProcess(outs, modelH, modelW, img0.rows, img0.cols, nmsThresh, confThresh, boxes, confs, ids);
}

// Check if target is in ROI
bool isInROI(const cv::Rect& box, const cv::Rect& roi) {
    cv::Point2f center(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
    return roi.contains(center);
}
t1_offboard_takeoff::clean_wild clean;
void clean_cb(const t1_offboard_takeoff::clean_wild::ConstPtr& msg)
{
    clean = *msg;
    is_clean_wild=msg->Is_Clean;//接受清0command 
    ROS_INFO("clean wild_msgs:Is_Clean=%d", msg->Is_Clean);  
     
}
int main(int argc, char **argv) {
    ros::init(argc,argv,"detection_node");
    ros::NodeHandle nh;
    ros::Rate(70);
    ros::Publisher dxy_pub=nh.advertise<t1_offboard_takeoff::dxy>("/detection_node/dxy",10);
    ros::Publisher wild_msg_pub=nh.advertise<t1_offboard_takeoff::wild>("/detection_node/wild_msg",10);
    ros::Publisher clean_stop_pub=nh.advertise<t1_offboard_takeoff::stop>("/stop_clean",10);
    ros::Publisher Detect_animals_pub=nh.advertise<t1_offboard_takeoff::Detect_animals>("/detection_node/Detect_animals",10);
    ////////////////////
    ros::Subscriber clean_msg_sub =nh.subscribe<t1_offboard_takeoff::clean_wild>("/should_clean",10,clean_cb);

    // Model path
    std::string modelPath = "/home/seeker/seeker_ws/src/t1_offboard_takeoff/onnx/99.onnx";
    
    // Labels
    std::vector<std::string> labels = {"E","K","M","T","W"};
    
    // Model parameters
    int modelH = 320;
    int modelW = 320;
    int nl = 3;
    int na = 3;
    std::vector<float> stride = {8.0f, 16.0f, 32.0f};
    
    // Anchor settings
    std::vector<std::vector<float>> anchors = {
        {10.0f, 13.0f, 16.0f, 30.0f, 33.0f, 23.0f},
        {30.0f, 61.0f, 62.0f, 45.0f, 59.0f, 119.0f},
        {116.0f, 90.0f, 156.0f, 198.0f, 373.0f, 326.0f}
    };
    
    // Prepare anchor grid
    std::vector<cv::Mat> anchorGrid(nl);
    for (int i = 0; i < nl; ++i) {
        cv::Mat anchorsMat(na, 2, CV_32F);
        float* ptr = anchorsMat.ptr<float>();
        for (size_t j = 0; j < anchors[i].size(); j += 2) {
            *ptr++ = anchors[i][j];
            *ptr++ = anchors[i][j+1];
        }
        anchorGrid[i] = anchorsMat;
    }
    
    // Initialize ONNX Runtime
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_detector");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
    
    // Load model
    Ort::Session session(nullptr);
    try {
        session = Ort::Session(env, modelPath.c_str(), sessionOptions);
        std::cout << "✅ Model loaded successfully: " << modelPath << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "❌ Failed to load model: " << e.what() << std::endl;
        return -1;
    }
    
    // Open camera
    cv::VideoCapture cap;
    for(int i=0;i<4;i++) {
        cap.open(i,cv::CAP_V4L2);
        if(cap.isOpened()) break;
    }
    if (!cap.isOpened()) {
        std::cerr << "❌ Failed to open camera" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);  // Resolution matching calibration parameters
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    int actual_w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "✅ Camera opened successfully (Resolution: " << actual_w << "x" << actual_h << ")" << std::endl;

    // -------------------------- Image rectification parameters --------------------------
    // Camera matrix (from calibration)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
      1269.42521,    0.     ,  382.20979,
            0.     , 1289.08152,  462.67358,
            0.     ,    0.     ,    1.  );
    // Distortion coefficients (from calibration)
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<-0.474476, 0.035295, -0.010755, 0.033492, 0.000000);
    
    // Precompute rectification maps (for real-time performance)
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(
        cameraMatrix,                  // Camera matrix
        distCoeffs,                    // Distortion coefficients
        cv::Mat(),                     // Rectification matrix (identity by default)
        cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(actual_w, actual_h), 1, cv::Size(actual_w, actual_h), 0),
        cv::Size(actual_w, actual_h),  // Output image size (same as input)
        CV_16SC2,                      // Map data type
        map1, map2                     // Output rectification maps
    );
    std::cout << "✅ Image rectification maps initialized" << std::endl;
    // ----------------------------------------------------------------------

    cv::Point img_center(actual_w / 2, actual_h / 2);  // Rectified image center
    std::cout << "ℹ️ Image center: (" << img_center.x << "," << img_center.y << ")" << std::endl;
    
    // ROI settings (user-defined region)
    cv::Rect roi(280, 130, 400, 430);
    // cv::Rect roi(413, 160, 400, 400);
    std::cout << "ℹ️ ROI position: (" << roi.x << "," << roi.y << "), Size: " << roi.width << "x" << roi.height << std::endl;
    
    // -------------------------- Counting logic variables --------------------------
    int total_count = 0;  // Total count
    const int CONTINUOUS_THRESHOLD = 6;  // 识别帧数判断
    const int MISS_THRESHOLD = 30;  // Frames to allow recount after disappearance (5 frames)
    // 每个类别的计数（key: 类别ID，value: 数量）
    std::map<int, int> class_counts;
    // Store continuous detection frames for each class
    std::map<int, int> class_continuous_counts;
    // Store counted classes (prevent duplicate counting)
    std::set<int> counted_classes;
    // Store missing frames for each class (for resetting counting state)
    std::map<int, int> class_missing_counts;
    // ----------------------------------------------------------------------
    
    // Main loop variables
    bool flagDet = true;                // Detection switch
    cv::Mat frame, undistorted_frame;   // frame: original image, undistorted_frame: rectified image
    std::vector<cv::Rect> boxes;        // Detection boxes
    std::vector<float> confs;           // Confidence scores
    std::vector<int> ids;               // Class IDs

    // FPS calculation
    auto last_fps_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    std::string fps_text = "FPS: --";

    cv::namedWindow("Detection Window (Rectified)", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Detection Window (Rectified)", actual_w, actual_h);
    
    while(ros::ok()) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "⚠️ Failed to get frame, retrying..." << std::endl;
            continue;
        }

        // Real-time image rectification
        cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);
        undistorted_frame=frame;
        frame_count++;

        // Calculate FPS
        auto now = std::chrono::steady_clock::now();
        float elapsed_fps = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_fps_time).count() / 1000.0f;
        
        if (elapsed_fps >= 1.0f) {
            float fps = frame_count / elapsed_fps;
            fps_text = "FPS: " + std::to_string((int)fps);
            last_fps_time = now;
            frame_count = 0;
        }
        cv::putText(undistorted_frame, fps_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // Draw ROI
        cv::rectangle(undistorted_frame, roi, ROI_COLOR, 2, cv::LINE_AA);

        // Initialize dxy
        dxy.dx = 0;
        dxy.dy = 0;
        dxy.detect_flag = 0;

        if (flagDet) {
            // 1. Perform inference
            inferImg(undistorted_frame, session, modelH, modelW, nl, na, stride, anchorGrid,
                    boxes, confs, ids, 0.4f, 0.6f);

            // 2. Filter targets outside ROI
            std::vector<cv::Rect> roi_boxes;
            std::vector<float> roi_confs;
            std::vector<int> roi_ids;
            for (size_t i = 0; i < boxes.size(); ++i) {
                if (isInROI(boxes[i], roi)) {
                    roi_boxes.push_back(boxes[i]);
                    roi_confs.push_back(confs[i]);
                    roi_ids.push_back(ids[i]);
                }
            }
            boxes = roi_boxes;
            confs = roi_confs;
            ids = roi_ids;

            // -------------------------- Core counting logic --------------------------
            // Record classes detected in current frame (for missing class judgment)
            std::set<int> current_classes(ids.begin(), ids.end());
            
            // 1. Update continuous counts and missing counts
            for (int class_id : current_classes) {
                // Class detected: reset missing count, increment continuous count
                class_missing_counts[class_id] = 0;
                class_continuous_counts[class_id]++;
            }
            
            for (auto& [class_id, cnt] : class_missing_counts) {
                // Class not detected: reset continuous count, increment missing count
                if (current_classes.find(class_id) == current_classes.end()) {
                    class_continuous_counts[class_id] = 0;
                    class_missing_counts[class_id]++;
                }
            }
            
            // 2. Check if continuous frames reach threshold and not counted
            for (int class_id : current_classes) {
                if (class_continuous_counts[class_id] >= CONTINUOUS_THRESHOLD && 
                    counted_classes.find(class_id) == counted_classes.end()) {
                    // Increment total count and mark as counted
                    //total_count++;
                    class_counts[class_id]++;
                    ROS_INFO("类别:class_id,数据存储%d", class_counts[class_id]);
                    counted_classes.insert(class_id);
                    std::cout << "Detected " << class_names[class_id] << ", Total count: " <<   class_counts[class_id] << std::endl;
                }
            }

            
            // 3. Reset classes missing for over 5 frames (allow recount)
            std::vector<int> to_remove;
            for (auto& [class_id, miss_cnt] : class_missing_counts) {
                if (miss_cnt >= MISS_THRESHOLD) {
                    counted_classes.erase(class_id);  // Remove from counted set
                    to_remove.push_back(class_id);    // Prepare to clear counting data
                }
            }
            for (int class_id : to_remove) {
                class_continuous_counts.erase(class_id);
                class_missing_counts.erase(class_id);
            }
            // ------------------------------------------------------------------
            ros::spinOnce(); 
            if(is_clean_wild==1)
            {
                ROS_INFO("CLEAN WILD_MSG!!!");
                // class_counts.clear();
                total_count = 0;
                class_counts.clear();
                counted_classes.clear();
                class_continuous_counts.clear();
                class_missing_counts.clear();
                
                is_clean_wild=0;
            }
            t1_offboard_takeoff::wild wild_msg;
            wild_msg.E=class_counts[0];
            wild_msg.K=class_counts[1];
            wild_msg.M=class_counts[2];
            wild_msg.T=class_counts[3];
            wild_msg.W=class_counts[4];
            wild_msg_pub.publish(wild_msg);
            if((wild_msg.E>0)||(wild_msg.K>0)||(wild_msg.M>0)||(wild_msg.T>0)||(wild_msg.W>0))
            {
                detect_animals.Is_Detected=1;
            }
            else
            {
                detect_animals.Is_Detected=0;
            }
            Detect_animals_pub.publish(detect_animals);
            // total_count = 0;
            // class_counts.clear();
            // counted_classes.clear();
            // class_continuous_counts.clear();
            // class_missing_counts.clear();
            //ROS_INFO("%d %d %d %d %d",wild_msg.E,wild_msg.K,wild_msg.M,wild_msg.T,wild_msg.W);

            // 3. Draw detection results
            for (size_t i = 0; i < boxes.size(); ++i) {
                int class_id = ids[i];
                // Add continuous frame info to label
                std::string label = labels[class_id] + 
                                   ": " + std::to_string(confs[i]).substr(0, 4) + 
                                   " (count: " + std::to_string(class_counts[class_id]) + ")";
                plotOneBox(boxes[i], undistorted_frame, class_id, label);
            }

            // 4. Calculate dx/dy (based on rectified image center)
            if (!boxes.empty()) {
                cv::Point target_center(
                    boxes[0].x + boxes[0].width / 2,
                    boxes[0].y + boxes[0].height / 2
                );
                dxy.dx = target_center.x - img_center.x;
                dxy.dy = target_center.y - img_center.y;
                dxy.detect_flag = 1;

                // Draw target center and arrow
                cv::circle(undistorted_frame, target_center, 5, TARGET_CENTER_COLOR, -1);
                cv::arrowedLine(undistorted_frame, img_center, target_center, ARROW_COLOR, 2, cv::LINE_AA, 0, 0.1);
            }

            // Draw image center
            cv::drawMarker(undistorted_frame, img_center, IMG_CENTER_COLOR, cv::MARKER_CROSS, 20, 2);
            cv::putText(undistorted_frame, "ImgCenter", cv::Point(img_center.x + 10, img_center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, IMG_CENTER_COLOR, 1);
        }
        
        // Draw total count
        cv::putText(undistorted_frame, "Total Count: " + std::to_string(total_count), 
                   cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, COUNT_COLOR, 2);
        
        // Publish dxy
        dxy_pub.publish(dxy);
        
        // Display rectified image
         // 缩小图像（核心缩放逻辑）
        cv::Mat resized_frame;
        const float SCALE_FACTOR = 0.5f;  // 调整缩放比例
        cv::resize(undistorted_frame, resized_frame, cv::Size(), SCALE_FACTOR, SCALE_FACTOR, cv::INTER_AREA);

        // 调整窗口大小以匹配缩放后的图像
        cv::resizeWindow("Detection Window (Rectified)", resized_frame.cols, resized_frame.rows);

        // 显示缩小后的图像
        cv::imshow("Detection Window (Rectified)", resized_frame);
            
        // Key handling
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;         // Exit program
        else if (key == 's') {         // Toggle detection
            flagDet = !flagDet;
            std::cout << "ℹ️ Detection " << (flagDet ? "enabled" : "disabled") << std::endl;
        } else if (key == 'r') {       // Reset count (press 'r')
            total_count = 0;
            class_counts.clear();
            counted_classes.clear();
            class_continuous_counts.clear();
            class_missing_counts.clear();
            std::cout << "🔄 Count reset" << std::endl;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
