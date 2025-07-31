#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<unistd.h>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

// 基本操作类似 stereo_euroc.cc

void save_to_csv(const std::vector<float>& vMem, const std::string& filename);
void SaveMapRecordToCSV(const std::vector<float>& Map_Record, const std::string& filename);
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

size_t mem_usage() {
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "VmRSS:", 6) == 0) {
            int len = strlen(line);

            const char* p = line;
            for (; std::isdigit(*p) == false; ++p) {}

            line[len - 3] = 0;
            result = atoi(p);

            break;
        }
    }

    fclose(file);
    return result;
}


int main(int argc, char **argv)
{

    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // NOTE 由于Kitti数据集的图像已经经过双目矫正的处理，所以这里就不需要再进行矫正的操作了

    vector<float> vMem;
    vector<float> Map_Record;
    float MapPointNum;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

        //统计映射地图点
        MapPointNum= SLAM.ReBacktheMappoint();
        Map_Record.push_back(MapPointNum);

        size_t vm = mem_usage();
        vMem.push_back((float)vm / 1024.0);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    //追踪完成，保存资源消耗信息
    save_to_csv(vMem,"/home/slam/CLionProjects/MES_SLAM/Memory_Use_MES_SLAM.csv");
    SaveMapRecordToCSV(Map_Record, "/home/slam/CLionProjects/MES_SLAM/Num_Point_MES_SLAM.csv");

    //统计映射地图点
    // int MapPointNum = SLAM.ReBacktheMappoint();
    cout << "共构建:\t" << MapPointNum << "\t个地图点"<< endl;

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // 以KITTI格式存储相机轨迹数据
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

// 类似 mono_kitti.cc， 不过是生成了双目的图像路径
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}


void save_to_csv(const std::vector<float>& vMem, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    file << "Iteration,Memory (MB)\n";  // 写入 CSV 头部
    for (size_t i = 0; i < vMem.size(); i++) {
        file << i + 1 << "," << vMem[i] << "\n";  // 写入数据
    }

    file.close();
    std::cout << "Memory usage data saved to " << filename << std::endl;
}

void SaveMapRecordToCSV(const std::vector<float>& Map_Record, const std::string& filename) {
    std::ofstream file(filename); // 以追加模式打开文件
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    file << "Iteration,Point(K)\n";  // 写入 CSV 头部
    for (size_t i = 0; i < Map_Record.size(); i++) {
        file << i + 1 << "," << Map_Record[i] << "\n";  // 写入数据
    }

    file.close();
    std::cout << "数据已保存至 " << filename << std::endl;
}
