#include "idetector.h"
#include "kcftracker.hpp"
// #include "multitracker.h"
#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"
#include "spdlog/stopwatch.h"
#include "itracker.h"
#include "realtracker.h"
#include <fstream>

// 在文件开头添加全局变量
bool mouseClickInit = false;  // 标记是否是鼠标点击触发的初始化
int click_x = 0;
int click_y = 0;
void drawRect(cv::Mat frame, cv::Rect r, cv::Scalar color = cv::Scalar(0,255,255))
{
    // 确保矩形在图像范围内
    r.x = std::max(0, std::min(r.x, frame.cols - 1));
    r.y = std::max(0, std::min(r.y, frame.rows - 1));
    r.width = std::min(r.width, frame.cols - r.x);
    r.height = std::min(r.height, frame.rows - r.y);

    // 如果矩形尺寸无效，直接返回
    if (r.width <= 0 || r.height <= 0) {
        return;
    }

    cv::Point tl{r.x, r.y};
    cv::Point bl{r.x, r.y + r.height};
    cv::Point tr{r.x + r.width, r.y};
    cv::Point br{r.x + r.width, r.y + r.height};

    int xlinelen = r.width/4;
    int ylinelen = r.height/4;
    int thickness = 3;

    // 绘制线段时确保不会超出图像边界
    auto drawSafeLine = [&frame](cv::Point p1, cv::Point p2, const cv::Scalar& color, int thickness) {
        p1.x = std::max(0, std::min(p1.x, frame.cols - 1));
        p1.y = std::max(0, std::min(p1.y, frame.rows - 1));
        p2.x = std::max(0, std::min(p2.x, frame.cols - 1));
        p2.y = std::max(0, std::min(p2.y, frame.rows - 1));
        cv::line(frame, p1, p2, color, thickness);
    };

    // 使用安全的绘制函数
    drawSafeLine(tl, cv::Point(tl.x + xlinelen, tl.y), color, thickness);
    drawSafeLine(tl, cv::Point(tl.x, tl.y + ylinelen), color, thickness);

    drawSafeLine(bl, cv::Point(bl.x + xlinelen, bl.y), color, thickness);
    drawSafeLine(bl, cv::Point(bl.x, bl.y - ylinelen), color, thickness);

    drawSafeLine(tr, cv::Point(tr.x - xlinelen, tr.y), color, thickness);
    drawSafeLine(tr, cv::Point(tr.x, tr.y + ylinelen), color, thickness);

    drawSafeLine(br, cv::Point(br.x - xlinelen, br.y), color, thickness);
    drawSafeLine(br, cv::Point(br.x, br.y - ylinelen), color, thickness);
}

void drawLostRect(cv::Mat frame, cv::Rect r)
{
    int s = 3;
    int w = r.width * s;
    int h = r.height * s;
    cv::Point pt(r.x+r.width/2, r.y+r.height/2);
    cv::Rect rc(pt.x - w/2, pt.y - h/2, w, h);
    drawRect(frame, rc, cv::Scalar(0,255,0));
}

cv::Rect box;//矩形对象
bool drawing_box = false;//记录是否在画矩形对象
bool box_complete = false;
cv::Point userPt;
int GateSize = 32;
int minIdx = -1;

bool contain = false;

bool trackerInited = false;
realtracker *rtracker = nullptr;
cv::Mat trackFrame;
cv::Mat dispFrame, trackRet, detFrame, trackRetByDet;
int trackOn;

int gateS = 64;

void onmouse(int event, int x, int y, int flag, void*)//鼠标事件回调函数，鼠标点击后执行的内容应在此
{
    // cv::Mat& image = *(cv::Mat*) img;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN://鼠标左键按下事件
        drawing_box = true;//标志在画框
        box = cv::Rect(x, y, 0, 0);//记录矩形的开始的点
        userPt.x = x;    //center point
        userPt.y = y;
        break;
    case cv::EVENT_MOUSEMOVE://鼠标移动事件
        if (drawing_box) {//如果左键一直按着，则表明在画矩形
            box.width = x - box.x;
            box.height = y - box.y;//更新长宽
        }
        break;
    case cv::EVENT_LBUTTONUP://鼠标左键松开事件
    {
        //不在画矩形
                            //这里好像没作用
        if (box.width < 0) {//排除宽为负的情况，在这里判断是为了优化计算，不用再移动时每次更新都要计算长宽的绝对值
            box.x = box.x + box.width;//更新原点位置，使之始终符合左上角为原点
            box.width = -1 * box.width;//宽度取正
        }
        if (box.height < 0) {//同上
            box.y = box.y + box.height;
            box.height = -1 * box.width;
        }
        // g_nCount++;
        // cv::Mat dst = image(box);
        // std::string str_save_name = std::to_string(g_nCount) + ".jpg";
        // cv::imwrite(str_save_name.c_str(), dst);
        printf("mouse btn up event, x:%d,y:%d,w:%d,h:%d\n", box.x, box.y, box.width, box.height);
        drawing_box = true;
        box_complete = true;
        if (trackOn) {
            if (rtracker) {
                rtracker->reset();
                rtracker->init( userPt, dispFrame, dispFrame);
                cv::rectangle(trackFrame, cv::Rect(userPt.x - gateS/2, userPt.y - gateS/2, gateS,gateS),cv::Scalar( 48,48,255 ), 2, 8 );
                trackerInited = true;
                cv::imshow("trackRet", trackFrame);
            }
        }
        
    }
        
        break;
    default:
        break;
    }
}

void onmouseTrack(int event, int x, int y, int flag, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (rtracker && trackOn) {
            cv::Point userPt(2*x, 2*y);
            rtracker->reset();
            rtracker->init(userPt, trackFrame, trackFrame);
            trackerInited = true;
            mouseClickInit = true;  // 设置鼠标点击标志
            click_x = x;
            click_y = y;
            spdlog::debug("Reinit tracker at point: ({}, {})", x, y);
        }
    }
}

void readTxt(std::vector<std::string>& testData){
    std::ifstream file("../test.txt");
    std::string line;
    std::vector<std::string> items;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        //std::string item;
        testData.emplace_back(line);
        // while (std::getline(ss, item, ',')) {
        //     items.push_back(item);
        // }
    }
    
    // 打印所有项
    // for (const auto& item : items) {
    //     std::cout << item << std::endl;
    // }
}

void writeToTxt(const std::string& filename, const std::vector<std::string>& data) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        spdlog::error("无法打开文件: {}", filename);
        return;
    }
    
    for (const auto& line : data) {
        outFile << line << std::endl;
    }
    outFile.close();
    spdlog::debug("数据已成功写入文件: {}", filename);
}

void autoTest(){
    
    int gateSize = 32;
    bool canAdjustGateSize = true;  // 添加标志位控制是否可以调整gateSize

    std::vector<std::string> testData;
    readTxt(testData);
    YAML::Node config = YAML::LoadFile("/home/rpdzkj/robusttracker-algodev/v0/config.yaml");
    int detOn = config["detection"].as<int>();
    trackOn = config["track"].as<int>();
    std::string engine = config["engine"].as<std::string>();
    std::string videopath = config["videopath"].as<std::string>();
    std::string irEngine = config["irengine"].as<std::string>();

    rtracker = new realtracker("/home/rpdzkj/robusttracker-algodev/v0/tracker.yaml");
    
    std::vector<std::string> results;  // 用于存储结果
    std::string gateSizeFile = "../output/gateSize.txt";
    std::ofstream gateSizeOutFile(gateSizeFile);
    for(auto& line : testData){
        std::stringstream ss(line);
        std::string item;

        // 用于存储分割后的数据
        std::string mp4_name;
        int orign_x, orign_y, start_frame;
        
        // 按逗号分割并依次读取数据
        std::getline(ss, mp4_name, ',');  // 读取mp4文件名
        
        std::getline(ss, item, ',');      // 读取x
        orign_x = std::stoi(item);


        std::getline(ss, item, ',');      // 读取y
        orign_y = std::stoi(item);


        std::getline(ss, item, ',');      // 读取start_frame
        start_frame = std::stoi(item);

        std::string mp4_path = "../../../vi/" + mp4_name + ".mp4";
        std::cout << "mp4_path: " << mp4_path << std::endl;
        cv::VideoCapture cap(mp4_path);
        if(!cap.isOpened())
        {
            printf("open failed\n");
            return;
        }
        cv::Mat frame;
        uint8_t trackerStatus[9];
        memset(trackerStatus, 0, 9);
        int nFrames = 0;

        // 创建结果文件

        std::string result_file = "../output/" + mp4_name+ "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y) + ".txt";
        spdlog::debug("result_file: {}", result_file);
        std::ofstream outFile(result_file);

        cv::namedWindow("trackRet");
        cv::setMouseCallback("trackRet", onmouseTrack);  // 设置鼠标回调
        canAdjustGateSize = true;
        int recordId = 0;
        while(1){
            
            cap >> frame;
            if(frame.empty())
                break;
            
            cv::resize(frame, frame, cv::Size(1280,720));
            trackFrame = frame.clone();  // 保存当前帧供回调函数使用
            if(nFrames < start_frame-1) {
                nFrames++;
                continue;
            }
            while (nFrames < start_frame){
                cv::Mat frame_clone = frame.clone();
                // 显示当前帧和gateSize信息
                cv::putText(frame_clone, "GateSize: " + std::to_string(gateSize), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
                // 画出预期的跟踪框
                cv::rectangle(frame_clone, cv::Rect(orign_x, orign_y, gateSize, gateSize), cv::Scalar(0,255,0), 2);
                cv::resize(frame_clone, frame_clone, cv::Size(640,360));
                cv::imshow("trackRet", frame_clone);
                
                // 等待按键输入
                char key = cv::waitKey(30);  // 使用30ms的延时使视频不会播放太快
                if(key == '[' && gateSize > 16) {
                    gateSize -= 8;
                    spdlog::debug("GateSize decreased to: {}", gateSize);
                }
                else if(key == ']' && gateSize < 128) {
                    gateSize += 8;
                    spdlog::debug("GateSize increased to: {}", gateSize);
                }
                else if(key == 32) {  // 空格键继续
                    canAdjustGateSize = false;
                    gateSizeOutFile << gateSize << std::endl;
                    rtracker->setGateSize(gateSize);
                }

                if(canAdjustGateSize == false) {  // 如果还在调整阶段，就不继续往下执行
                    break;
                }
            }
            nFrames++;
            if(nFrames < start_frame){
                
                continue;
            }

            if (nFrames == start_frame){
                int x = orign_x + gateSize/2;
                if(x + gateSize/2 > 1280) continue;
                int y = orign_y + gateSize/2;
                if(y + gateSize/2 > 720) continue;
                std::cout << "start_frame: " << nFrames << "start_coord: " << x << " ," << y << std::endl;
                cv::Point userPt(x, y);

                if (trackOn) {
                    if (rtracker) {
                        rtracker->reset();
                        rtracker->init( userPt, frame, frame);
                        //cv::rectangle(frame, cv::Rect(userPt.x - gateS/2, userPt.y - gateS/2, gateS,gateS),cv::Scalar( 48,48,255 ), 2, 8 );
                        trackerInited = true;
                        //cv::imshow("trackRet", frame);
                    }
                }
            }
            
            int center_x,center_y;
            cv::Rect trackRect;
            if(trackOn) {
                if (trackerInited) {
                    rtracker->update(frame, frame, trackerStatus, center_x, center_y, trackRect);
                    
                    if(rtracker->trackerLost())
                    {
                        drawLostRect(frame, trackRect);
                    }
                    else
                        drawRect(frame, trackRect);

                    spdlog::debug("tracker status:{}", trackerStatus[4]);

                    cv::resize(frame, frame, cv::Size(640,360));
                }

                cv::putText(frame, std::to_string(nFrames), cv::Point(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
                cv::putText(frame, "Click to reinit tracker", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1);
                cv::putText(frame, "Press 'n' for next frame", cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1);
                cv::putText(frame, "Press 'q' to quit current video", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1);
                cv::putText(frame, "video name: " + mp4_name + " " + std::to_string(orign_x) + " " + std::to_string(orign_y), cv::Point(50, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1);
                cv::imshow("trackRet", frame);
            }
            
            // 等待'n'键来显示下一帧
            while(true) {
                char c = cv::waitKey(0);  // 无限等待按键
                if(c == 'n') {  // 只有按'n'键才会继续
                    break;
                }
                else if(c == 'q') {  // 按'q'键退出当前视频
                    spdlog::debug("Quit current video");
                    goto next_video;  // 跳转到处理下一个视频
                }
            }
            if(mouseClickInit){
                recordId++;
                mouseClickInit = false;  // 重置标志
                line = std::to_string(nFrames) + "," + std::to_string(click_x*2 - gateSize/2) + "," + std::to_string(click_y*2 - gateSize/2) + "," + std::to_string(gateSize) + "," + std::to_string(gateSize);
                outFile << line << std::endl;
                continue;
            }
            line = std::to_string(nFrames) + "," + std::to_string(trackRect.x) + "," + std::to_string(trackRect.y) + "," + std::to_string(trackRect.width) + "," + std::to_string(trackRect.height);
            outFile << line << std::endl;
            recordId++;
            
        }
        
next_video:  // 添加标签用于跳转
        nFrames = 0;
        outFile.close();
        spdlog::debug("数据已成功写入文件: {}", result_file);

        // 打印解析后的数据
        spdlog::debug("mp4: {}, x: {}, y: {}, start_frame: {}", 
                     mp4_name, orign_x, orign_y, start_frame);
        
    }
    gateSizeOutFile.close();
}

struct TrackData{
    int frameNum;
    int x;
    int y;
    int width;
    int height;
};

void readTxtShow(std::string filePath){

    std::ifstream file(filePath);
    std::string fileName = filePath.substr(filePath.find_last_of("/")+1,-1);
    std::string mp4Name = fileName.substr(0,fileName.find_first_of("_")) + ".mp4";
    std::string mp4Path = "../../../vi/" + mp4Name;
    
    cv::VideoCapture cap(mp4Path);
    if(!cap.isOpened())
    {
        printf("open failed\n");
        return;
    }
    cv::Mat frame;

    std::string line;
     std::vector<TrackData> trackDataList;
    // 读取txt文件
    while(std::getline(file, line)){
        std::stringstream ss(line);
        std::string item;
        TrackData trackData;
        
        std::getline(ss, item, ',');
        trackData.frameNum = std::stoi(item);
        std::getline(ss, item, ',');
        trackData.x = std::stoi(item);
        std::getline(ss, item, ',');
        trackData.y = std::stoi(item);
        std::getline(ss, item, ',');
        trackData.width = std::stoi(item);
        std::getline(ss, item, ',');
        trackData.height = std::stoi(item);
        trackDataList.push_back(trackData);
    }
    int frameCount = 0;
    while(1){
        frameCount++;
        cap >> frame;
        if(frame.empty())
            break;
        if (frameCount < trackDataList[0].frameNum){
            continue;
        }
        cv::resize(frame, frame, cv::Size(1280,720));
        cv::rectangle(frame, cv::Rect(trackDataList[frameCount].x, trackDataList[frameCount].y, trackDataList[frameCount].width, trackDataList[frameCount].height), cv::Scalar(0,0,255), 2);
        cv::resize(frame, frame, cv::Size(640,360));
        cv::imshow("trackRet", frame);
        cv::waitKey(1);
    }
}

// 添加一个辅助函数用于安全地计算坐标
cv::Point getSafePoint(int x, int y, int gateSize, const cv::Mat& frame) {
    // 确保中心点不会导致框超出图像边界
    int safe_x = std::max(gateSize/2, std::min(x, frame.cols - gateSize/2));
    int safe_y = std::max(gateSize/2, std::min(y, frame.rows - gateSize/2));
    return cv::Point(safe_x, safe_y);
}

void autoCompare(){
    float diffPercent = 0.8;
    int gateSize = 32;

    std::vector<std::string> testData;
    readTxt(testData);
    YAML::Node config = YAML::LoadFile("/home/rpdzkj/robusttracker-algodev/v0/config.yaml");
    int detOn = config["detection"].as<int>();
    trackOn = config["track"].as<int>();
    std::string engine = config["engine"].as<std::string>();
    std::string videopath = config["videopath"].as<std::string>();
    std::string irEngine = config["irengine"].as<std::string>();

    rtracker = new realtracker("/home/rpdzkj/robusttracker-algodev/v0/tracker.yaml");
    
    std::vector<std::string> results;  // 用于存储结果

    // 读取gateSize.txt文件，获取gateSize列表
    // std::vector<int> gateSizeList;  
    // std::string gateSizeFile = "../output/gateSize.txt";
    // std::ifstream gateSizeFileStream(gateSizeFile);
    // std::string gateSizeLine;
    // while(std::getline(gateSizeFileStream, gateSizeLine)){
    //     gateSizeList.push_back(std::stoi(gateSizeLine));
    // }

    std::ofstream outResultFile;
    std::string resultFile = "../output/result.txt";
    outResultFile.open(resultFile);
    int index = 0;
    //std::vector<int> gateSizeList;
    std::unordered_map<std::string, int> gateSizeMap;
    // 获取gateSize列表
    for(auto& line : testData){
        std::stringstream ss(line);
        std::string item;
    
        // 用于存储分割后的数据
        std::string mp4_name;
        int orign_x, orign_y, start_frame;
        
        // 按逗号分割并依次读取数据
        std::getline(ss, mp4_name, ',');  // 读取mp4文件名
        
        std::getline(ss, item, ',');      // 读取x
        orign_x = std::stoi(item);


        std::getline(ss, item, ',');      // 读取y
        orign_y = std::stoi(item);

        std::string txtOuputPath = "../output/" + mp4_name + "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y) + ".txt";
        std::ifstream txtOuputFile(txtOuputPath);
        std::string txtLine;
        if (std::getline(txtOuputFile, txtLine)) {
            std::stringstream txtLineStream(txtLine);
            // 跳过前三个数字
            for(int i = 0; i < 3; i++) {
                std::getline(txtLineStream, item, ',');
            }
            // 读取第四个数字
            std::getline(txtLineStream, item, ',');
            int fourthNumber = std::stoi(item);
            gateSizeMap[mp4_name + "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y)] = fourthNumber;
            spdlog::debug("Fourth number: {}", fourthNumber);
        }
        txtOuputFile.close();

    }

    int lostTrackNum = 0;
    for(auto& line : testData){
        std::stringstream ss(line);
        std::string item;
        
        

        // 用于存储分割后的数据
        std::string mp4_name;
        int orign_x, orign_y, start_frame;
        
        // 按逗号分割并依次读取数据
        std::getline(ss, mp4_name, ',');  // 读取mp4文件名
        
        std::getline(ss, item, ',');      // 读取x
        orign_x = std::stoi(item);


        std::getline(ss, item, ',');      // 读取y
        orign_y = std::stoi(item);


        std::getline(ss, item, ',');      // 读取start_frame
        start_frame = std::stoi(item);

        gateSize = gateSizeMap[mp4_name + "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y)];
        
        // 读取每一帧的标签
        std::string labelFile = "../output/" + mp4_name + "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y) + ".txt";
        std::ifstream labelFileStream(labelFile);
        std::string labelLine;
        // 读取每一帧的标签
        std::vector<TrackData> trackDataList;
        while(std::getline(labelFileStream, labelLine)){
            try {
                TrackData trackData;
                std::stringstream labelLineStream(labelLine);
                std::string item;
                
                // 检查每一行是否为空
                if(labelLine.empty()) {
                    spdlog::warn("跳过空行");
                    continue;
                }

                // 逐个读取并检查每个字段
                if(!std::getline(labelLineStream, item, ',')) {
                    spdlog::error("读取frameNum失败");
                    continue;
                }
                if(item.empty()) {
                    spdlog::error("frameNum为空");
                    continue;
                }
                trackData.frameNum = std::stoi(item);

                if(!std::getline(labelLineStream, item, ',')) {
                    spdlog::error("读取x坐标失败");
                    continue;
                }
                if(item.empty()) {
                    spdlog::error("x坐标为空");
                    continue;
                }
                trackData.x = std::stoi(item);

                if(!std::getline(labelLineStream, item, ',')) {
                    spdlog::error("读取y坐标失败");
                    continue;
                }
                if(item.empty()) {
                    spdlog::error("y坐标为空");
                    continue;
                }
                trackData.y = std::stoi(item);

                if(!std::getline(labelLineStream, item, ',')) {
                    spdlog::error("读取width失败");
                    continue;
                }
                if(item.empty()) {
                    spdlog::error("width为空");
                    continue;
                }
                trackData.width = std::stoi(item);

                if(!std::getline(labelLineStream, item, ',')) {
                    spdlog::error("读取height失败");
                    continue;
                }
                if(item.empty()) {
                    spdlog::error("height为空");
                    continue;
                }
                trackData.height = std::stoi(item);

                trackDataList.push_back(trackData);
            } catch (const std::exception& e) {
                spdlog::error("解析行数据时发生错误: {} 行内容: {}", e.what(), labelLine);
                continue;  // 跳过这一行，继续处理下一行
            }
        }

        // 检查是否成功读取到数据
        if(trackDataList.empty()) {
            spdlog::error("标签文件未读取到有效数据");
            continue;  // 跳过当前视频
        }

        std::string mp4_path = "../../../vi/" + mp4_name + ".mp4";
        std::cout << "mp4_path: " << mp4_path << std::endl;
        cv::VideoCapture cap(mp4_path);
        if(!cap.isOpened())
        {
            printf("open failed\n");
            return;
        }
        cv::Mat frame;
        uint8_t trackerStatus[9];
        memset(trackerStatus, 0, 9);
        int nFrames = 0;

        // 创建结果文件
        cv::namedWindow("trackRet");
        //cv::setMouseCallback("trackRet", onmouseTrack);  // 设置鼠标回调
        int fileIndex = 0;
        int diffFrame = 0;
        while(1){
            cap >> frame;
            if(frame.empty())
                break;
            
            cv::resize(frame, frame, cv::Size(1280,720));
            trackFrame = frame.clone();  // 保存当前帧供回调函数使用
            nFrames++;  
            if (nFrames < start_frame){
                
                continue;
            }
            if (nFrames == start_frame){
                rtracker->setGateSize(gateSize);
                // 使用安全的坐标计算
                cv::Point safePoint = getSafePoint(
                    trackDataList[1].x + gateSize/2,
                    trackDataList[1].y + gateSize/2,
                    gateSize,
                    frame
                );
                
                if (trackOn) {
                    if (rtracker) {
                        rtracker->reset();
                        rtracker->init(safePoint, frame, frame);
                        trackerInited = true;  
                    }
                }
            }
            
            int center_x,center_y;
            cv::Rect trackRect;
            if(trackOn) {
                if (trackerInited) {
                    rtracker->update(frame, frame, trackerStatus, center_x, center_y, trackRect);
                    if(fileIndex < trackDataList.size()){
                        // 绘制标签框（红色）
                        cv::Rect labelRect(trackDataList[fileIndex].x, 
                                         trackDataList[fileIndex].y,
                                         trackDataList[fileIndex].width,
                                         trackDataList[fileIndex].height);
                        drawRect(frame, labelRect, cv::Scalar(0,0,255));  // 红色框
                        
                        int x_diff = std::abs(trackRect.x - trackDataList[fileIndex].x);
                        int y_diff = std::abs(trackRect.y - trackDataList[fileIndex].y);
                        
                        // 在画面左上角显示差异信息
                        // cv::putText(frame, "X_diff: " + std::to_string(x_diff), 
                        //           cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 
                        //           0.8, cv::Scalar(0,0,255), 1);
                        // cv::putText(frame, "Y_diff: " + std::to_string(y_diff), 
                        //           cv::Point(50, 110), cv::FONT_HERSHEY_SIMPLEX, 
                        //           0.6, cv::Scalar(0,0,255), 1);
                        // cv::putText(frame, "Frame: " + std::to_string(nFrames), 
                        //           cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 
                        //           0.6, cv::Scalar(0,0,255), 1);
                        // cv::putText(frame, "DiffFrames: " + std::to_string(diffFrame), 
                        //           cv::Point(50, 70), cv::FONT_HERSHEY_SIMPLEX, 
                        //           0.6, cv::Scalar(0,0,255), 1);
                        cv::putText(frame, "gateSize: " + std::to_string(gateSize), 
                                  cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 
                                  0.8, cv::Scalar(0,0,255), 2);
                        cv::putText(frame, "mp4_name: " + mp4_name + "_" + std::to_string(orign_x) + "_" + std::to_string(orign_y), 
                                  cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 
                                  0.8, cv::Scalar(0,0,255), 2);        
                        
                        spdlog::debug("y_diff:{}", y_diff);
                        spdlog::debug("x_diff:{}", x_diff);
                        if(x_diff > diffPercent * trackDataList[fileIndex].width || y_diff > diffPercent * trackDataList[fileIndex].height){
                            diffFrame++;
                            spdlog::debug("diffFrame:{}", diffFrame);
                        }
                        
                        fileIndex++;
                    }

                    else{
                        break;
                    }
                    
                    // 绘制跟踪框（黄色）
                    if(rtracker->trackerLost())
                    {
                        drawLostRect(frame, trackRect);
                    }
                    else
                        drawRect(frame, trackRect);  // 默认黄色(0,255,255)

                    spdlog::debug("tracker status:{}", trackerStatus[4]);

                    cv::resize(frame, frame, cv::Size(640,360));
                }
                cv::imshow("trackRet", frame);
            }
            cv::waitKey(1);
            // 等待'n'键来显示下一帧
            // while(true) {
            //     char c = cv::waitKey(0);  // 无限等待按键
            //     if(c == 'n') {  // 只有按'n'键才会继续
            //         break;
            //     }
            //     else if(c == 'q') {  // 按'q'键退出当前视频
            //         spdlog::debug("Quit current video");
            //         goto next_video;  // 跳转到处理下一个视频
            //     }
            // }

        }
        
        
        
next_video:  // 添加标签用于跳转
        
        if(diffFrame > 10){
            lostTrackNum++;
        }

        outResultFile << mp4_name << "_" << std::to_string(orign_x) << "_" << std::to_string(orign_y) << ":" << diffFrame << "," << nFrames << std::endl;
        nFrames = 0;
        diffFrame = 0;
        // 打印解析后的数据
        spdlog::debug("mp4: {}, x: {}, y: {}, start_frame: {}", 
                     mp4_name, orign_x, orign_y, start_frame);
        
    }
    outResultFile << "lostTrackNum: " << lostTrackNum << ","
                     << "lostTrackRate: " << lostTrackNum * 1.0 / testData.size() << std::endl;
    outResultFile.close();
}



int main(int argc, char*argv[]){
    if (setenv("DISPLAY", "192.168.4.1:0.0", 1) != 0) {
        std::cerr << "Failed to set DISPLAY environment variable." << std::endl;
        return 1;
    }
    spdlog::set_level(spdlog::level::debug); 
    autoCompare();
    //autoTest();
    //readTxtShow("../output/1_760_263.txt");
    return 0;
}

// int main(int argc, char*argv[])
// {
//     //autoTest();
//     if (setenv("DISPLAY", "192.168.4.1:0.0", 1) != 0) {
//         std::cerr << "Failed to set DISPLAY environment variable." << std::endl;
//         return 1;
//     }
//     //int waitVAL =  argc > 1 ? 10 : 0;
//     int waitVAL = 0;
//     spdlog::set_level(spdlog::level::debug); // Set global log level to debug

//     YAML::Node config = YAML::LoadFile("/home/rpdzkj/robusttracker-algodev/v0/config.yaml");
//     int detOn = config["detection"].as<int>();
//     trackOn = config["track"].as<int>();
//     std::string engine = config["engine"].as<std::string>();
//     std::string videopath = config["videopath"].as<std::string>();
//     std::string irEngine = config["irengine"].as<std::string>();

//     rtracker = new realtracker("/home/rpdzkj/robusttracker-algodev/v0/tracker.yaml");
//     // rtracker->setGateSize(gateS);
//     gateS = rtracker->osdw;

//     cv::VideoCapture cap(videopath);
//     if(!cap.isOpened())
//     {
//         printf("open failed\n");
//         return 0;
//     }

//     std::vector<bbox_t> boxs;

//     cv::Mat frame;
//     int nFrames = 0;

//     cv::namedWindow("trackRet");
//     cv::setMouseCallback("trackRet", onmouse);

//     uint8_t trackerStatus[9];
//     memset(trackerStatus, 0, 9);

//     bbox_t detRet[200];

//     cv::VideoWriter video;//("output.avi", fourcc,30.0, cv::Size(640, 512));
//     bool recvid = false;

//     while(1)
//     {
//         cap >> frame;
//         // frame = cv::imread("/home/rpdzkj/3/model/1.jpg");
//         if(frame.empty())
//             break;

//         // frame = cv::imread("/space/data/123.PNG");
//         cv::resize(frame, frame, cv::Size(1280,720));

//         trackFrame = frame.clone();
//         detFrame = frame.clone();
//         dispFrame = frame.clone();
//         trackRetByDet = frame.clone();

//         printf("=====nframe:%d======\n", nFrames);

//         int center_x,center_y;
//         cv::Rect trackRect;
//         int boxes_count;
//         if(trackOn) {
//             if (trackerInited) {
//                 rtracker->update(trackFrame, trackFrame, trackerStatus, center_x, center_y, trackRect);

//                  if(rtracker->trackerLost())
//                 {
//                     drawLostRect(trackFrame, trackRect);
//                 }
//                 else
//                     drawRect(trackFrame, trackRect);

//                 spdlog::debug("tracker status:{}", trackerStatus[4]);

//                 cv::resize(trackFrame, trackFrame, cv::Size(640,360));
//             }

//             cv::putText(trackFrame, std::to_string(nFrames), cv::Point(150, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, cv::LINE_AA);

            
//             cv::imshow("trackRet", trackFrame);
//             if(recvid)
//             {
//                 video.write(frame);
//             }
//         }
        

//         if(detOn)
//         {
//             rtracker->runDetectorOut(detFrame, detRet, boxes_count);


//             for (int i = 0; i < boxes_count; ++i)
//             {
//                 bbox_t &box = detRet[i];
//                     drawRect(detFrame, cv::Rect(cv::Point(box.x, box.y), cv::Point(box.x + box.w, box.y + box.h)), cv::Scalar{255,0,0});
//             }
//             // cv::imshow("final-detRet", detFrame);
//         }

        
//         char c = cv::waitKey(waitVAL);
//         if(c == 'g')
//             waitVAL = 1;
//         else if(c == 's')
//             waitVAL = 0;
//         else if(c == 'r')
//         {
//             recvid = true;
//             video.open(std::to_string(userPt.x) + "-" + std::to_string(userPt.y) + "_.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0, cv::Size(1280,720));
//             spdlog::debug("video record start");
//         }
//         else if(c == 't')
//         {
//             recvid = false;
//             video.release();
//             spdlog::debug("video record stop");
//         }

//         nFrames++;
//     }

//     return 0;
// }