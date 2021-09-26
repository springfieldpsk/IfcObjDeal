#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <array>
#include <cmath>
#include <set>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#define GLM_FORCE_RADIANS
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

namespace IfcObjDealSpace {
    typedef std::vector<std::vector<uint32_t> > GroupList;
    typedef std::vector<uint32_t> Group;
};

class UnionFind {
public:

    UnionFind() {}
    UnionFind(uint32_t len) {
        IndexParent.resize(len);
        for (size_t i = 0; i < len; i++) {
            IndexParent[i] = i;
        }
    }

    void AddCon(uint32_t u, uint32_t v) {
        if (u < 0 || v < 0 || u >= IndexParent.size() || v >= IndexParent.size()) return;
        uint32_t uu = FindParent(u);
        uint32_t vv = FindParent(v);
        if (uu == vv) return;
        IndexParent[uu] = vv;
    }

    std::vector<std::vector<uint32_t> > GetUnionGroup() {
        IfcObjDealSpace::GroupList res;
        std::unordered_map<uint32_t,uint32_t>GroupVis;

        for (size_t i = 0; i < IndexParent.size(); i++) {
            uint32_t u = FindParent(IndexParent[i]);
            if (!GroupVis.count(u)) {
                GroupVis[u] = res.size();
                std::vector<uint32_t>tmp;
                res.push_back(tmp);
            }
            res[GroupVis[u]].push_back(i);
        }

        return res;
    }
    // 获取下标集

private:
    std::vector<uint32_t>IndexParent;
    uint32_t FindParent(uint32_t u) {
        if (IndexParent[u] == u) return u;
        return IndexParent[u] = FindParent(IndexParent[u]);
    }
}; // 并查集类

struct VertexSet
{
    const float ThresholdWidth = 0.2;
    const float ThresholdHeight = 0.2;

    std::vector<glm::vec3>Vertices; // 点集
    std::unordered_map<glm::vec3, bool>VerticesVis; // 确定点是否在点集内存在
    std::string Name;   // 点集所属物体名
    glm::vec3 NormalVector; // 唯一法向量
    std::array<glm::vec3, 2> BaseVertices; // 基础点集
    std::array<glm::vec3, 2> AABBBox; // AABB包围盒

    bool UseFlag; // 使用标记 确定是否符合要求
    bool TestCancelFlag; // 取消合并测试标志
    bool JulCancelFlag; // 取消判断测试标志

    inline void AddIn(const glm::vec3& v) {
        if (VerticesVis.count(v)) return;
        VerticesVis[v] = true;
        Vertices.push_back(v);
    } // 加入点，通过判断加入点是否存在防止重添加

    VertexSet(const std::string& _Name, const glm::vec3& _Nv, const glm::vec3& v1, const glm::vec3& v2,const float& ThsW,const float& ThsH) :Name(_Name), NormalVector(_Nv),ThresholdWidth(ThsW),ThresholdHeight(ThsH) {
        AddIn(v1);
        AddIn(v2);

        BaseVertices[0] = v1;
        BaseVertices[1] = v2;

        AABBBox[0] = { std::min(v1.x,v2.x),std::min(v1.y,v2.y),std::min(v1.z,v2.z) };
        AABBBox[1] = { std::max(v1.x,v2.x),std::max(v1.y,v2.y),std::max(v1.z,v2.z) };

        UseFlag = true;
        TestCancelFlag = false;
        JulCancelFlag = false;
    } // 初始化构造函数 将初始点加入点集并构建AABB包围盒

    void AddVector(glm::vec3 v) {
        AddIn(v);
        AABBBox[0] = { std::min(v.x,AABBBox[0].x),std::min(v.y,AABBBox[0].y),std::min(v.z,AABBBox[0].z) };
        AABBBox[1] = { std::max(v.x,AABBBox[1].x),std::max(v.y,AABBBox[1].y),std::max(v.z,AABBBox[1].z) };
    }// 加入矢量 同时更新AABB盒

    bool CheckVector(glm::vec3 v) {
        if (v == BaseVertices[0] || v == BaseVertices[1]) return true;
        glm::vec3 Normal = glm::cross(BaseVertices[0] - v, BaseVertices[1] - v);
        Normal = glm::normalize(Normal);
        return (CheckNormalVector(Normal) || Normal == glm::vec3(0, 0, 0));
    }// 检测点是否与点集同平面 若非基础两点之一 则利用叉积计算法向量 并与面法向量相比较 确定是否同面 存在共线情况 所以也需要与0向量相比较

    inline bool CheckNormalVector(glm::vec3 v) {
        return (v == NormalVector || v == -NormalVector);
    }// 比较法向量

    void CheckThreshold(const std::unordered_map<std::string, bool>& ThresholdName) {
        glm::vec3 Vec = (AABBBox[0] - AABBBox[1]);
        float VecList[3] = { abs(Vec.x),abs(Vec.y),abs(Vec.z) };
        std::sort(VecList, VecList + 3);
        //for (int i = 0; i < 3; i++) std::cout << VecList[i] << ' ';
        //std::cout << '\n';
        //std::cout << Name << std::endl;
        UseFlag = !(VecList[2] < ThresholdHeight || VecList[1] < ThresholdWidth || ThresholdName.count(Name));
    }// 阈值判断 通过AABB包围盒确定是否符合阈值范围 通过支持名剔除

    void Output() {
        for (auto v : Vertices) {
            std::cout << v.x << ' ' << v.y << ' ' << v.z << std::endl;
        }
        puts("");
        //std::cout << "AABB BOX:" << '\n';
    }
};
// 点集类

struct VerticesGroup
{
    std::vector<glm::vec3>Vertices; // 点集
    std::string Name;   // 点集所属物体名

    VerticesGroup(VertexSet v) {
        Vertices = v.Vertices;
        Name = v.Name;
    }
    VerticesGroup(VertexSet v, IfcObjDealSpace::Group g) {
        Name = v.Name;
        for (const auto& i : g) {
            Vertices.push_back(v.Vertices[i]);
        }
    }
};

class IfcObjDeal {
private:
   
    const std::string MODEL_PATH;
    const std::string WRITE_PATH;
    const float ThresholdWidth;
    const float ThresholdHeight;
    const float eps = 1e-6;

    std::unordered_map<std::string, bool>ThresholdName;
    std::unordered_map<std::string, uint32_t>VerticesCntMap;
    std::unordered_map<std::string, bool>CancelOutMap;
    std::unordered_map<std::string, bool>CancelJulMap;

    std::vector<VertexSet> VerticesSets;
    std::vector<VerticesGroup> VerticesGroups;
    std::vector<std::string> ThresholdNameList;
    std::vector<std::string> CancelOutNameList;
    std::vector<std::string> CancelJudNameList;

public:
    IfcObjDeal(const std::string m_path,const std::string w_path, const float& width, const float& height) :MODEL_PATH(m_path),WRITE_PATH(w_path),ThresholdHeight(height), ThresholdWidth(width) {

    };

    inline bool CheckEqualFloat(float a, float b) {
        return abs(a - b) < eps;
    }

    void ReadList(const std::vector<std::string>& ThrList, const std::vector<std::string>&CancelOList, const std::vector<std::string>& CancelJList) {
        ThresholdNameList = ThrList;
        CancelJudNameList = CancelJList;
        CancelOutNameList = CancelOList;
    }

    void initIfcObjDeal() {

        for (const auto& Name : ThresholdNameList) {
            ThresholdName[Name] = true;
        }

        for (const auto& Name : CancelOutNameList) {
            CancelOutMap[Name] = true;
        }

        for (const auto& Name : CancelJudNameList) {
            CancelJulMap[Name] = true;
        }
    }

    void LoadObj() {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, MODEL_PATH.c_str())) {
            throw std::runtime_error(warn + err);
        }

        for (size_t s = 0; s < shapes.size(); s++) {
            // 遍历各物体
            size_t index_offset = 0;
            // 确定偏移值
            size_t VeticesSetsBegin = VerticesSets.size();

            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
                // 遍历物体上各面
                size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

                if (shapes[s].name[0] == '!') {
                    index_offset += fv;
                    continue;
                }
                std::vector<glm::vec3> FaceIndices;

                for (size_t v = 0; v < fv; v++) {
                    // 遍历面上各点
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                    tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                    tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                    glm::vec3 nv = { vx,vy,vz };
                    FaceIndices.push_back(nv);
                }

                if (FaceIndices.size() < 3) {
                    throw std::runtime_error("Obj File Error");
                }
                // 若面的总点数小于3，则抛出错误

                glm::vec3 Normal = glm::cross(FaceIndices[0] - FaceIndices[1], FaceIndices[0] - FaceIndices[2]);
                Normal = glm::normalize(Normal);
                // 生成法向量

                bool FacceAddFlag = false;

                for (size_t i = VeticesSetsBegin; i < VerticesSets.size(); i++) {
                    if (VerticesSets[i].CheckNormalVector(Normal)) {
                        uint32_t AddCnt = 0;
                        for (const auto& v : FaceIndices) {
                            if (VerticesSets[i].CheckVector(v)) {
                                AddCnt++;
                            }
                        } // 判断该面在点集内的存在情况

                        if (AddCnt == fv) {
                            FacceAddFlag = true;
                            for (const auto& v : FaceIndices) VerticesSets[i].AddVector(v);
                            break;
                        }// 若点数一致 则认为面在点集内存在 将面加入点集内 
                    }
                }

                if (!FacceAddFlag) {
                    VerticesSets.push_back(VertexSet(shapes[s].name, Normal, FaceIndices[0], FaceIndices[1], ThresholdWidth, ThresholdHeight));
                    for (size_t i = 2; i < fv; i++) {
                        VerticesSets[VerticesSets.size() - 1].AddVector(FaceIndices[i]);
                    }
                }
                // 若已进入点集点数与面上总点数不符，则新建一个以该三角面为基础的点集

                index_offset += fv;
            }
        }
    } // 载入Obj文件至私有格式

    void preReject() {
        //std::cout << "PreReject Name:" << '\n';
        for (size_t i = 0; i < VerticesSets.size(); i++) {
            VerticesSets[i].CheckThreshold(ThresholdName);

            if (CancelOutMap.count(VerticesSets[i].Name)) {
                if (VerticesSets[i].UseFlag) {
                    VerticesGroup tmp = VerticesGroup(VerticesSets[i]);
                    VerticesGroups.push_back(tmp);
                }
                //std::cout << VerticesSets[i].Name << '\n';
                VerticesSets[i].TestCancelFlag = true;
            }

            if (CancelJulMap.count(VerticesSets[i].Name)) {
                VerticesSets[i].JulCancelFlag = true;
            }
        }
    }
    // 预剔除

    void InitVerGroup() {
        for (size_t i = 0; i < VerticesSets.size(); i++) {
            //std::cout << "!:" << VerticesSets[i].Name << ' ' << VerticesSets[i].Vertices.size() <<' '<< VerticesSets[i].UseFlag<<' '<< VerticesSets[i].TestCancelFlag <<'\n';
            if (!VerticesSets[i].UseFlag || VerticesSets[i].TestCancelFlag) continue;
            //std::cout << "!:"<<VerticesSets[i].Name << ' '<<VerticesSets[i].Vertices.size()<<'\n';
            for (size_t j = 0; j < VerticesSets.size(); j++) {
                if (VerticesSets[j].UseFlag || VerticesSets[j].JulCancelFlag) continue;

                for (auto v : VerticesSets[j].Vertices) {
                    if (VerticesSets[i].CheckVector(v)) {
                        VerticesSets[i].AddIn(v);
                    }
                }
            }
            //std::cout << "!:" << VerticesSets[i].Name << ' ' << VerticesSets[i].Vertices.size() << '\n';
            // 合并点集 将剔除出的点集中的点加入未剔除的点集

            std::unordered_map<glm::vec3, bool>CoordinateVis;
            UnionFind* uF = new UnionFind(VerticesSets[i].Vertices.size());
            std::vector<VertexSet> tmp;

            for (const auto& v : VerticesSets[i].Vertices) {
                std::vector<std::pair<float, uint32_t> >VerticesGroupAxisX;
                std::vector<std::pair<float, uint32_t> >VerticesGroupAxisY;
                std::vector<std::pair<float, uint32_t> >VerticesGroupAxisZ;
                glm::vec3 AxisX = { 0, v.y,v.z };
                glm::vec3 AxisY = { v.x,   0,v.z };
                glm::vec3 AxisZ = { v.x, v.y,  0 };
                bool AxisXExist = CoordinateVis.count(AxisX);
                bool AxisYExist = CoordinateVis.count(AxisY);
                bool AxisZExist = CoordinateVis.count(AxisZ);

                for (size_t j = 0; j < VerticesSets[i].Vertices.size(); j++) {
                    glm::vec3 u = VerticesSets[i].Vertices[j];
                    if (!AxisXExist && CheckEqualFloat(u.y, v.y) && CheckEqualFloat(u.z, v.z)) VerticesGroupAxisX.push_back(std::make_pair(u.x, j));
                    if (!AxisYExist && CheckEqualFloat(u.x, v.x) && CheckEqualFloat(u.z, v.z)) VerticesGroupAxisY.push_back(std::make_pair(u.y, j));
                    if (!AxisZExist && CheckEqualFloat(u.x, v.x) && CheckEqualFloat(u.y, v.y)) VerticesGroupAxisZ.push_back(std::make_pair(u.z, j));
                }

                if (!AxisXExist) {
                    CoordinateVis[AxisX] = true;
                    std::sort(VerticesGroupAxisX.begin(), VerticesGroupAxisX.end());
                    for (size_t j = 0; j < VerticesGroupAxisX.size() - 1; j += 2) {
                        uF->AddCon(VerticesGroupAxisX[j].second, VerticesGroupAxisX[j + 1].second);
                    }
                }

                if (!AxisYExist) {
                    CoordinateVis[AxisY] = true;
                    std::sort(VerticesGroupAxisY.begin(), VerticesGroupAxisY.end());
                    for (size_t j = 0; j < VerticesGroupAxisY.size() - 1; j += 2) {
                        uF->AddCon(VerticesGroupAxisY[j].second, VerticesGroupAxisY[j + 1].second);
                    }
                }

                if (!AxisZExist) {
                    CoordinateVis[AxisZ] = true;
                    std::sort(VerticesGroupAxisZ.begin(), VerticesGroupAxisZ.end());
                    for (size_t j = 0; j < VerticesGroupAxisZ.size() - 1; j += 2) {
                        uF->AddCon(VerticesGroupAxisZ[j].second, VerticesGroupAxisZ[j + 1].second);
                    }
                }
            }

            IfcObjDealSpace::GroupList Groups = uF->GetUnionGroup();
            delete uF;
            // 定义一条线段是只有起点和终点的直线，以此建立拓扑关系，生成点集内点群

            for (const auto& Group : Groups) {
                VerticesGroup tmp = VerticesGroup(VerticesSets[i], Group);
                VerticesGroups.push_back(tmp);
            }
            // 分割点群 生成新点集
        }
    }
    // 生成点群

    void OutputVerSets() {
        std::cout << "VerticesSets Name\n";
        for (const auto& v : VerticesSets) {
            if (!v.UseFlag) continue;
            std::cout << v.Name << ' ' << v.Vertices.size() << '\n';
        }
        std::cout << '\n';

        std::cout << "VerticesGroups Name\n";
        for (const auto& v : VerticesGroups) {
            std::cout << v.Name << ' ' << v.Vertices.size() << '\n';
        }
        std::cout << '\n';
    }
    void VerGroupReName() {
        for (size_t i = 0; i < VerticesGroups.size(); i++) {

            std::string VertexName = VerticesGroups[i].Name;
            if (VerticesCntMap.count(VertexName)) {
                VerticesCntMap[VertexName] ++;
                VertexName = VertexName + "-" + std::to_string(VerticesCntMap[VertexName]);
            }
            else VerticesCntMap[VertexName] = 0;
            VerticesGroups[i].Name = VertexName;

            //std::cout << VerticesGroups[i].Name << ' ' << VerticesGroups[i].Vertices.size() << std::endl;
        }
    }
    // 重命名

    void WriteToCsv() {
        std::ofstream dstFile(WRITE_PATH, std::ios::out);
        if (!dstFile) {
            throw std::runtime_error("写入文件失败");
        }
        uint32_t cnt = 0;
        for (const auto& v : VerticesGroups) {
            cnt = std::max(cnt, v.Vertices.size());
        }

        dstFile << "面编号,";

        for (size_t i = 0; i < cnt; i++) {
            if (i) dstFile << ",";
            dstFile << ",坐标" + std::to_string(i + 1) << ",";
        }
        dstFile << std::endl;

        for (size_t i = 0; i < VerticesGroups.size(); i++) {
            dstFile << VerticesGroups[i].Name << ",";
            for (const auto& v : VerticesGroups[i].Vertices) {
                dstFile << v.x << "," << v.y << "," << v.z << ",";
            }
            dstFile << std::endl;
        }

        dstFile.close();
    }

    void mainLoop() {

        initIfcObjDeal();
        LoadObj();
        preReject();
        //OutputVerSets();
        InitVerGroup();
        VerGroupReName();
        WriteToCsv();
    }
};

class BuildIfcObjInstance {
private:
    const std::string CFG_PATH;
public:
    BuildIfcObjInstance(const std::string& _path) :CFG_PATH(_path) {
        if (!JudgeFile(CFG_PATH, "cfg")) {
            throw std::runtime_error("cfg file error");
        }
    }
    bool JudgeFile(const std::string& _path, const std::string& Bak) {
        std::string bak = "";
        for (const auto& v : _path) {
            if (v == '.') bak.clear();
            else bak.push_back(v);
        }
        // cout<<bak<<endl;
        return bak == Bak;
    }
    IfcObjDeal* RunCFG() {
        std::string SRC_PATH;
        std::string DST_PATH;
        float THR_W, THR_H;

        std::ifstream CFGFile(CFG_PATH, std::ios::in);
        CFGFile >> SRC_PATH >> DST_PATH >> THR_W >> THR_H;

        if (!JudgeFile(SRC_PATH, "obj")) {
            throw std::runtime_error(SRC_PATH + "obj file error");
        }
        if (!JudgeFile(DST_PATH, "csv")) {
            throw std::runtime_error(DST_PATH + "csv file error");
        }

        std::vector<std::string> ThrList;
        std::vector<std::string> CancelOLis;
        std::vector<std::string> CancelJList;
        uint32_t flag = 0;
        std::string tmp;

        while (CFGFile >> tmp) {
            if (tmp == "THR") flag = 1;
            else if (tmp == "COL") flag = 2;
            else if (tmp == "CJL") flag = 3;
            else {
                if (flag == 1) ThrList.push_back(tmp);
                else if (flag == 2) CancelOLis.push_back(tmp);
                else if (flag == 3) CancelJList.push_back(tmp);
            }
        }

         std::cout<<SRC_PATH<<' '<<DST_PATH<<' '<<THR_W<<' '<<THR_H<<std::endl;
         std::cout<<"THr:";
         for(auto v : ThrList) std::cout<<v<<' ';
         std::cout<<std::endl;
         std::cout<<"COL:";
         for(auto v : CancelOLis) std::cout<<v<<' ';
         std::cout<<std::endl;
         std::cout<<"CJL:";
         for(auto v : CancelJList) std::cout<<v<<' ';
         std::cout<<std::endl;
        IfcObjDeal* instance = new IfcObjDeal(SRC_PATH, DST_PATH, THR_W, THR_H);
        instance->ReadList(ThrList, CancelOLis, CancelJList);
        return instance;
    }
};

int main(int arg,char** args) {

    if (arg != 3 || strcmp(args[1], "--CFG")) {
        std::cerr << "输入参数错误 请使用客户端打开" << std::endl;
        return 0;
    }

    BuildIfcObjInstance* instance = new BuildIfcObjInstance(args[2]);
    instance->RunCFG();
    IfcObjDeal* Instance = instance->RunCFG();
    
    try {
        Instance->mainLoop();
    }
    catch (const std::exception& e) {
        delete Instance;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    delete Instance;
    std::cout << "处理结束" << '\n';
    return EXIT_SUCCESS;
}