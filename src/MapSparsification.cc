#include "MapSparsification.h"
#include <unistd.h>

namespace ORB_SLAM2 {
    MapSparsification::MapSparsification(const string &strSettingsFile,
                                         Map *mpMap) :
            mbStopRequested(false), mbStopped(true), mnId(0), mGRBEnv(GRBEnv(true)),
            mbFinishRequested(false), mbFinished(true), mmap(mpMap){
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        mnMinNum = fsSettings["Sparsification.N"];
        mfLambda = fsSettings["Sparsification.Lambda"];
        mfGridLambda = fsSettings["Sparsification.GridLambda"];
        mnWindowLength = fsSettings["Sparsification.WindowLength"];
        cout << endl << "*****************************************" << endl;
        cout << "Map Sparsification settings: " << endl;
        cout << "Sparsification.N: " << mnMinNum << endl;
        cout << "Sparsification.Lambda: " << mfLambda << endl;
        cout << "Sparsification.GridLambda: " << mfGridLambda << endl;
        cout << "Sparsification.WindowLength: " << mnWindowLength << endl;
        cout << "*****************************************" << endl;
        mGRBEnv.start();
    }

    void MapSparsification::Run() {
        // 标记当前任务未完成
        mbFinished = false;
        //开启地图点稀疏化模块
        while (true) {
            // 处理新关键帧
            if (CheckNewKeyFrames()) {
                {     //检查是否有新的关键帧
                    //线程同步，修改 mbStopped 状态，确保多个线程不会冲突。
                    unique_lock<mutex> lock2(mMutexStop);
                    //加锁
                    mbStopped = false;
                }
                // 获取最新的关键帧。
                // vector<shared_ptr<KeyFrame> > vpKFs = GetLastestKeyFrames();
                vector<KeyFrame*> vpKFs = GetLastestKeyFrames();
                // 对这些关键帧进行 稀疏化处理，减少地图中的冗余信息，提高效率。
                Sparsifying(vpKFs);
                {
                    // 再次加锁后,表示Sparsifying完成
                    unique_lock<mutex> lock2(mMutexStop);
                    mbStopped = true;
                }
            }

            if (CheckFinish()) {
                // 获取所有关键帧。
                vector<KeyFrame*> vAllKeyFrames = mmap->GetAllKeyFrames();
                vector<KeyFrame*> vRemainKeyFrames;
                for (KeyFrame* pKFi : vAllKeyFrames) {
                    // 筛选出未稀疏化的关键帧进行处理
                    if (!pKFi->mbSparsified) {
                        vRemainKeyFrames.push_back(pKFi);  // ✅ 直接存储裸指针
                    }
                }
                // 再次进行稀疏化。
                Sparsifying(vRemainKeyFrames);
                for (KeyFrame* pKFi : vRemainKeyFrames) {
                    // 清除低质量的描述子，进一步优化地图存储。
                    pKFi->EraseBadDescriptor();
                }

                break;
            }
            usleep(3000);
        }
        SetFinish();
    }

    void MapSparsification::Sparsifying(vector<KeyFrame*> &vpKFs) {
        // 获取所有关键帧，并初始化优化器
        vector<KeyFrame*> vAllKFs = mmap->GetAllKeyFrames();
        mnId++;
        GRBModel model = GRBModel(mGRBEnv);
        std::vector<GRBVar> vxs; // all parameters for every MP
        GRBLinExpr expr = 0; // cost function，优化目标函数
        vector<MapPoint*> vLocalMapPoints; // 存储所有待优化的 MapPoint。
        long unsigned int index = 0;
        int nMaxObsevation = 0;

        for (KeyFrame* pKFi : vpKFs) {
            vector<MapPoint*> vMPs = pKFi->GetMapPointMatches();
            for (MapPoint* pMPi : vMPs) {
                if (!pMPi || pMPi->isBad())
                    continue;
                int nObservation = pMPi->Observations();
                if (nObservation > nMaxObsevation)
                    nMaxObsevation = nObservation;
            }
        }

        // 遍历 vpKFs 构建优化问题
        for (KeyFrame* pKF : vpKFs) {
            GRBLinExpr expr_cons = 0;
            std::vector<std::vector<std::vector<size_t>>> grids = pKF->GetFeatureGrids();
            pKF->mnMapSaprsificationId = mnId;

            for (vector<vector<size_t>> &gridi : grids) {
                for (vector<size_t> &grid : gridi) {
                    if (grid.empty())
                        continue;
                    GRBLinExpr expr_cons_grid = 0;
                    bool bValidCell = false;
                    for (size_t i : grid) {
                        MapPoint* pMP = pKF->GetMapPoint(i);
                        if (pMP && (!pMP->isBad())) {
                            if (pMP->mnMapSparsificationId != mnId) {
                                vLocalMapPoints.push_back(pMP);
                                pMP->mnMapSparsificationId = mnId;
                                // 创建二元变量 x ，构造优化变量
                                GRBVar x = model.addVar(0, 1, 0, GRB_BINARY);
                                // 计算权重：观测次数少的 MapPoint 代价更高，即更容易被移除
                                float coef = (nMaxObsevation - pMP->Observations());
                                expr += coef * x;
                                // 存入变量 vxs
                                vxs.push_back(x);
                                pMP->mnIndexForSparsification = index;
                                index++;
                                expr_cons += x;
                                expr_cons_grid += x;
                                bValidCell = true;
                            } else {
                                long unsigned int iForComp = pMP->mnIndexForSparsification;
                                expr_cons += vxs[iForComp];
                                expr_cons_grid += vxs[iForComp];
                                bValidCell = true;
                            }
                        }
                    }
                    // 条件限制，即每一个网格至少有一个 MapPoint，不能全部删除
                    if (bValidCell) {
                        GRBVar th_grid = model.addVar(0, 1, 0, GRB_BINARY);
                        expr_cons_grid += th_grid;
                        model.addConstr(expr_cons_grid, '>', 1);
                        expr += mfGridLambda * th_grid;
                    }
                }
            }

            // 设置 th 变量，确保关键帧至少保留 mnMinNum 个地图点
            GRBVar th = model.addVar(0, 1000, 0, GRB_INTEGER);
            expr += mfLambda * th;
            expr_cons += th;
            model.addConstr(expr_cons, '>', mnMinNum); // 添加约束条件
        }

        // 处理额外关键帧观测约束
        map<KeyFrame*, int> extraNum;
        map<KeyFrame*, GRBLinExpr> extraConstrints;
        for (auto pMPi: vLocalMapPoints) {
            long unsigned int iForComp = pMPi->mnIndexForSparsification;
            // const map<KeyFrame*, tuple<int, int>> observations = pMPi->GetObservations();
            const map<KeyFrame*, size_t> observations = pMPi->GetObservations();
            for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
                KeyFrame* pKFi = mit->first;  // 修改为裸指针
                if (pKFi->mnMapSaprsificationId != mnId) {
                    if (extraNum.count(pKFi)) {
                        extraNum[pKFi]++;  // 使用裸指针作为键
                        extraConstrints[pKFi] += vxs[iForComp];  // 使用裸指针作为键
                    } else {
                        extraNum[pKFi] = 1;
                        extraConstrints[pKFi] = vxs[iForComp];
                    }
                }
            }
        }


        // 确保外部 KeyFrame 仍然有足够的 MapPoint，避免破坏已有的地图质量
        for (auto it = extraNum.begin(), itend = extraNum.end(); it != itend; it++) {
            KeyFrame* pKFi = it->first;
            float nTotal = pKFi->GetNumberMPs();
            float nMini = (float) it->second / nTotal * mnMinNum;
            GRBVar th = model.addVar(0, 1000, 0, GRB_INTEGER);
            expr += mfLambda * th;
            model.addConstr(extraConstrints[pKFi] + th, '>', nMini);
        }

        // 优化器结果
        model.setObjective(expr, GRB_MINIMIZE);
        model.set(GRB_IntParam_OutputFlag, 0);
        float MIPGap = 0.0020;
        model.set(GRB_DoubleParam_MIPGap, MIPGap);
        model.optimize();

        // 打印优化后的最小目标函数值
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            double objVal = model.get(GRB_DoubleAttr_ObjVal);  // 获取最小目标函数值
            std::cout << "Optimized objective value: " << objVal << std::endl;
        } else {
            std::cout << "Optimization did not find an optimal solution." << std::endl;
        }

        for (int i = 0; i < vLocalMapPoints.size(); ++i) {
            MapPoint* pMPi = vLocalMapPoints[i];
            long unsigned int iForComp = pMPi->mnIndexForSparsification;
            double keep = vxs[iForComp].get(GRB_DoubleAttr_X);
            if (keep <= 0) {
                pMPi->SetBadFlag();
            }
        }

        for (KeyFrame* pKFi : vpKFs) {
            mpLoopClosing->InsertSparsifiedKeyFrame(pKFi);
        }
    }


    vector<KeyFrame*> MapSparsification::GetLastestKeyFrames() {
        // 确保多个线程不会同时修改
        unique_lock<mutex> lock(mMutexNewKFs);

        vector<KeyFrame*> vKFs;  // 用于返回的 KeyFrame 裸指针列表

        if (mvpNewKeyFrames.size() > mnWindowLength) {
            // 提取前 mnWindowLength 个裸指针
            for (size_t i = 0; i < mnWindowLength; i++) {
                vKFs.push_back(mvpNewKeyFrames[i]);
            }
            // 移除已处理的裸指针
            mvpNewKeyFrames.erase(mvpNewKeyFrames.begin(), mvpNewKeyFrames.begin() + mnWindowLength);
        } else {
            // 提取所有剩余的裸指针
            for (KeyFrame* pKFi : mvpNewKeyFrames) {
                vKFs.push_back(pKFi);
            }
            mvpNewKeyFrames.clear();
        }

        return vKFs;
    }



    void MapSparsification::InsertKeyFrame(KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        mvpNewKeyFrames.push_back(pKF);
    }

    bool MapSparsification::CheckNewKeyFrames() {
        unique_lock<mutex> lock2(mMutexStop);
        unique_lock<mutex> lock(mMutexNewKFs);
        return mvpNewKeyFrames.size() > 10 && (!mbStopRequested);
    }

    void MapSparsification::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void MapSparsification::RequestStop() {
        unique_lock<mutex> lock2(mMutexStop);
        mbStopRequested = true;
        cout << "Map Sparsification STOP" << endl;
    }

    void MapSparsification::Release() {
        unique_lock<mutex> lock2(mMutexStop);
        mbStopRequested = false;
        cout << "Map Sparsification RELEASE" << endl;
    }

    bool MapSparsification::isStopped() {
        unique_lock<mutex> lock2(mMutexStop);
        return mbStopped;
    }

    void MapSparsification::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool MapSparsification::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void MapSparsification::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool MapSparsification::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

}
