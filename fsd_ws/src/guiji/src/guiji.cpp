#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include "casadi/casadi.hpp"
#include <memory>
#include <std_msgs/Float64MultiArray.h>
#include <utility>
#include <sensor_msgs/PointCloud.h>

using namespace std;
using namespace chrono;
using namespace casadi;
using namespace Eigen;
#define M_PI 3.14159265358979323846

using namespace std;
using namespace chrono;
using namespace casadi;
using namespace Eigen;
using namespace std::chrono_literals;
#define M_PI 3.14159265358979323846


std::vector<std::pair<double, double>> tong_red;    //红桶输入
std::vector<std::pair<double, double>> tong_blue;    //蓝桶输入


std::vector<std::pair<double, double>> tong_r;    //红桶输入
std::vector<std::pair<double, double>> tong_b;    //蓝桶输入

std::vector<std::pair<double, double>> lujing; //lujing

std::vector<std::pair<double, double>> lujing2; //lujing

std::vector<std::pair<double, double>> tyquanxian;

std::vector<std::pair<double, double>> weilai_lujing;    //weilai路径点

double dt = 0.5;

// 定义距离计算函数
double distance(std::pair<double, double> p1, std::pair<double, double> p2) {
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return std::sqrt(dx * dx + dy * dy);
}


// 计算 tong_r 和 tong_b 之间的最短距离
double calculateShortestDistance(const std::vector<std::pair<double, double>>& points1, const std::vector<std::pair<double, double>>& points2) {
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& p1 : points1) {
        for (const auto& p2 : points2) {
            double dist = distance(p1, p2);
            if (dist < minDistance) {
                minDistance = dist;
            }
        }
    }

    return minDistance;
}


// 移动贝塞尔曲线上的所有点
std::vector<std::pair<double, double>> moveBezierCurve(
    const std::vector<std::pair<double, double>>& bezierPoints,
    double dx, double dy) {
    std::vector<std::pair<double, double>> movedPoints;

    for (const auto& point : bezierPoints) {
        movedPoints.emplace_back(point.first + dx, point.second + dy);
    }

    return movedPoints;
}


// 定义比较函数，返回距离较短的点
std::pair<double, double> bijiaodian(std::pair<double, double> p1, std::pair<double, double> p2, std::pair<double, double> p0) {
    double dist1 = distance(p1, p0);
    double dist2 = distance(p2, p0);

    if (dist1 < dist2) {
        return p1;
    }
    else {
        return p2;
    }
}

// 计算二阶贝塞尔曲线
std::vector<std::pair<double, double>> bezierCurveQuadratic(const std::vector<std::pair<double, double>>& controlPoints, int numPoints = 100) {
    std::vector<std::pair<double, double>> curvePoints;

    if (controlPoints.size() < 3) {
        return curvePoints; // 至少需要 3 个点
    }

    for (int j = 0; j <= numPoints; ++j) {
        double t = static_cast<double>(j) / numPoints; // 参数 t

        double x = std::pow(1 - t, 2) * controlPoints[0].first +
            2 * (1 - t) * t * controlPoints[1].first +
            std::pow(t, 2) * controlPoints[2].first;

        double y = std::pow(1 - t, 2) * controlPoints[0].second +
            2 * (1 - t) * t * controlPoints[1].second +
            std::pow(t, 2) * controlPoints[2].second;

        curvePoints.emplace_back(x, y);
    }

    return curvePoints;
}

// 计算三阶贝塞尔曲线
std::vector<std::pair<double, double>> bezierCurveCubic(const std::vector<std::pair<double, double>>& controlPoints, int numPoints = 100) {
    std::vector<std::pair<double, double>> curvePoints;

    if (controlPoints.size() < 4) {
        return curvePoints; // 至少需要 4 个点
    }

    for (int j = 0; j <= numPoints; ++j) {
        double t = static_cast<double>(j) / numPoints; // 参数 t

        double x = std::pow(1 - t, 3) * controlPoints[0].first +
            3 * std::pow(1 - t, 2) * t * controlPoints[1].first +
            3 * (1 - t) * t * t * controlPoints[2].first +
            t * t * t * controlPoints[3].first;

        double y = std::pow(1 - t, 3) * controlPoints[0].second +
            3 * std::pow(1 - t, 2) * t * controlPoints[1].second +
            3 * (1 - t) * t * t * controlPoints[2].second +
            t * t * t * controlPoints[3].second;

        curvePoints.emplace_back(x, y);
    }

    return curvePoints;
}

// 计算四阶贝塞尔曲线
std::vector<std::pair<double, double>> bezierCurveQuartic(const std::vector<std::pair<double, double>>& controlPoints, int numPoints = 100) {
    std::vector<std::pair<double, double>> curvePoints;

    if (controlPoints.size() < 5) {
        return curvePoints; // 至少需要 5 个点
    }

    for (int j = 0; j <= numPoints; ++j) {
        double t = static_cast<double>(j) / numPoints; // 参数 t

        double x = std::pow(1 - t, 4) * controlPoints[0].first +
            4 * std::pow(1 - t, 3) * t * controlPoints[1].first +
            6 * std::pow(1 - t, 2) * t * t * controlPoints[2].first +
            4 * (1 - t) * t * t * t * controlPoints[3].first +
            t * t * t * t * controlPoints[4].first;

        double y = std::pow(1 - t, 4) * controlPoints[0].second +
            4 * std::pow(1 - t, 3) * t * controlPoints[1].second +
            6 * std::pow(1 - t, 2) * t * t * controlPoints[2].second +
            4 * (1 - t) * t * t * t * controlPoints[3].second +
            t * t * t * t * controlPoints[4].second;

        curvePoints.emplace_back(x, y);
    }

    return curvePoints;
}


// 通用贝塞尔曲线拟合函数（动态选择阶数）
std::vector<std::pair<double, double>> bezierCurve(const std::vector<std::pair<double, double>>& controlPoints, int numPoints = 100) {
    if (controlPoints.size() == 3) {
        return bezierCurveQuadratic(controlPoints, numPoints); // 二阶贝塞尔曲线
    }
    else if (controlPoints.size() >= 4) {
        return bezierCurveCubic(controlPoints, numPoints); // 三阶贝塞尔曲线
    }

    return {}; // 点数不足，返回空结果
}


// 排序函数，用于对点进行排序
void sortPointsByDistance(std::vector<std::pair<double, double>>& points, const std::pair<double, double>& referencePoint) {
    std::sort(points.begin(), points.end(), [&referencePoint](const std::pair<double, double>& a, const std::pair<double, double>& b) {
        return distance(a, referencePoint) < distance(b, referencePoint);
        });
}


void xianchenghanshu()
{
     // 记录上一次刷新时刻
    auto lastTime = std::chrono::steady_clock::now();
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "SFML Point");
    while (1)
    {
        // 处理事件
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        // 获取当前时间
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);
        
        
        if(elapsed.count() % 10==0)
        {
        window.clear(); // 清除屏幕，避免重复绘制

        // 绘制背景矩形
        sf::RectangleShape rectangle;
        rectangle.setSize(sf::Vector2f(71.25, 35.75));
        rectangle.setFillColor(sf::Color::White);
        rectangle.setPosition((-1.05 + 20) * 25, (-0.715 + 20) * 25);
        window.draw(rectangle);

        // 绘制中心点
        sf::CircleShape point(5); // 中心点
        point.setFillColor(sf::Color::Green);
        point.setPosition((0 + 20) * 25, (0 + 20) * 25);
        window.draw(point);

        // 绘制相邻锥桶之间的连线函数
        auto drawLinesBetweenPoints = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
            for (size_t i = 1; i < points.size(); ++i)
            {
                sf::Vertex line[] =
                {
                    sf::Vertex(sf::Vector2f((points[i - 1].first + 20) * 25, (-points[i - 1].second + 20) * 25), color),
                    sf::Vertex(sf::Vector2f((points[i].first + 20) * 25, (-points[i].second + 20) * 25), color)
                };
                window.draw(line, 2, sf::Lines);
            }
            };

        // 绘制红锥桶的点和连线
        for (const auto& num : tong_red)
        {
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Red);
            point.setPosition((num.first + 20) * 25, (-num.second + 20) * 25);
            window.draw(point);
        }
        //drawLinesBetweenPoints(tong_red, sf::Color::Red);

        // 绘制蓝锥桶的点和连线
        for (const auto& num : tong_blue)
        {
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Blue);
            point.setPosition((num.first + 20) * 25, (-num.second + 20) * 25);
            window.draw(point);
        }
        //drawLinesBetweenPoints(tong_blue, sf::Color::Blue);

        // 绘制筛选后红锥桶的点和连线
        /*for (const auto& num : tong_r)
        {
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Red);
            point.setPosition((num.first + 20) * 25, (-num.second + 20) * 25);
            window.draw(point);
        }*/
        //drawLinesBetweenPoints(tong_r, sf::Color::Red);

        // 绘制筛选后蓝锥桶的点和连线
        /*for (const auto& num : tong_b)
        {
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Blue);
            point.setPosition((num.first + 20) * 25, (-num.second + 20) * 25);
            window.draw(point);
        }*/
        //drawLinesBetweenPoints(tong_b, sf::Color::Blue);

        for (const auto& num : lujing2)
        {
            sf::CircleShape point(5);
            point.setFillColor(sf::Color::Green);
            point.setPosition((num.first + 20) * 25, (-num.second + 20) * 25);
            window.draw(point);

        }

        // 绘制贝塞尔曲线的辅助函数
        auto drawBezierCurve = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
            for (size_t i = 1; i < points.size(); ++i) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f((points[i - 1].first + 20) * 25, (-points[i - 1].second + 20) * 25), color),
                    sf::Vertex(sf::Vector2f((points[i].first + 20) * 25, (-points[i].second + 20) * 25), color)
                };
                window.draw(line, 2, sf::Lines);
            }
            };

        // 计算 tong_r 和 tong_b 的最短距离 d
        double d = calculateShortestDistance(tong_r, tong_b);




        // 计算 tong_r 和 tong_b 的贝塞尔曲线
        std::vector<std::pair<double, double>> bezier_r = bezierCurve(tong_r);
        std::vector<std::pair<double, double>> bezier_b = bezierCurve(tong_b);
        std::vector<std::pair<double, double>> bezier_l = bezierCurveQuartic(lujing2);



        // 绘制筛选后红锥桶的贝塞尔曲线
        drawBezierCurve(bezier_r, sf::Color::Yellow);

        // 绘制筛选后蓝锥桶的贝塞尔曲线
        drawBezierCurve(bezier_b, sf::Color::White);


        // 绘制贝塞尔曲线
        //drawBezierCurve(lujing, sf::Color::Green); // 四阶贝塞尔曲线绘制为绿色

        //
       // drawBezierCurve(bezier_l, sf::Color::Cyan);

        drawBezierCurve(tyquanxian, sf::Color::Red);

        window.display(); // 显示绘制内容
        }
    }
}

//计算点到直线距离，没有除以sqrt（a^2+b^2）
double diantoxian(const std::pair<double, double>& x, const  std::pair<double, double>& x1, const  std::pair<double, double>& x2)
{
    double A = x2.second - x1.second;
    double B = x1.first - x2.first;
    double C = x1.second * x2.first - x1.first * x2.second;
    return abs(A * x.first + B * x.second + C);
}

//寻找里某个点最近的两个点，输出下标
vector<int> xunzuijin(vector<std::pair<double, double>>& xy, vector<bool>& used, std::pair<double, double>& dian)
{
    vector<int> result;
    int num = 0;
    for (int i = 0; i < used.size(); i++)
    {
        if (used[i]) num++;
    }

    if (num == 0) return {};
    if (num == 1)
    {
        for (int i = 0; i < used.size(); i++)
        {
            if (used[i])
            {
                result.push_back(i);
            }
        }
        return result;
    }

    double tmpzhi1 = 99999999;
    double tmpzhi2 = 100000000;
    int tmpbiao1 = 0;
    int tmpbiao2 = 0;

    for (int i = 0; i < xy.size(); i++)
    {
        if (used[i])
        {
            //cout << "distance" << distance(dian, xy[i]) << endl;
            if (distance(dian, xy[i]) < tmpzhi2)
            {

                if (distance(dian, xy[i]) < tmpzhi1)
                {
                    tmpzhi2 = tmpzhi1;
                    tmpbiao2 = tmpbiao1;

                    tmpzhi1 = distance(dian, xy[i]);
                    tmpbiao1 = i;
                }
                else
                {
                    tmpzhi2 = distance(dian, xy[i]);
                    tmpbiao2 = i;
                }



            }

            //cout << "1:" << tmpbiao1 << "    2:" << tmpbiao2 << endl;
        }
    }

    return { tmpbiao1,tmpbiao2 };
}

//用来判断新加的路径点是否合理
bool pandian(vector < std::pair<double, double>>& xy_good)
{
    int num = xy_good.size();

    if (num <= 2) return true;

    std::pair<double, double> shiliang1 = { xy_good[num - 1].first - xy_good[num - 2].first,xy_good[num - 1].second - xy_good[num - 2].second };
    std::pair<double, double> shiliang2 = { xy_good[num - 2].first - xy_good[num - 3].first,xy_good[num - 2].second - xy_good[num - 3].second };

    double cosjiao = (shiliang1.first * shiliang2.first + shiliang1.second * shiliang2.second) / (sqrt(shiliang1.first * shiliang1.first + shiliang1.second * shiliang1.second) *
        sqrt(shiliang2.first * shiliang2.first + shiliang2.second * shiliang2.second));

    //cout << "num" << num << "cos" << cosjiao << endl;

    //角度大于+-45认为不可行
    if (cosjiao > 0.7071)
    {
        return true;
    }
    else
    {
        return false;
    }

}

//用来计算路径线
void zhuitongluxian(vector<std::pair<double, double>>& xy, vector<std::pair<double, double>>& xy_good, vector<bool>& used, int num, int jishu)
{
    if (num == 0) return;
    if (xy_good.size() == 0)
    {
        xy_good.push_back(xy[0]);
        used[0] = 0;
        num--;
        jishu++;
    }
    if (xy_good.size() == 1 && num != 0)
    {
        vector<int> zuijindian = xunzuijin(xy, used, xy_good[0]);

        if (zuijindian.size() == 0) return;
        if (zuijindian.size() == 1)
        {
            xy_good.push_back(xy[zuijindian[0]]);
            used[zuijindian[0]] = 0;
            num--;
            jishu++;
        }
        else
        {
        if (abs(diantoxian(xy[zuijindian[0]], xy_good[0], { 0,xy_good[0].second }) - diantoxian(xy[zuijindian[1]], xy_good[0], { 0,xy_good[0].second })) < 0.1)
        {
            xy_good.push_back(xy[zuijindian[0]]);
            used[zuijindian[0]] = 0;
            num--;
            jishu++;
        }
        else
        {
            if (diantoxian(xy[zuijindian[0]], xy_good[0], { 0,xy_good[0].second }) < diantoxian(xy[zuijindian[1]], xy_good[0], { 0,xy_good[0].second }))
            {
                //cout << "111" << zuijindian[0] << endl;
                xy_good.push_back(xy[zuijindian[0]]);
                used[zuijindian[0]] = 0;
                num--;
                jishu++;
            }
            else
            {
                //cout << "222" << zuijindian[1] << endl;
                xy_good.push_back(xy[zuijindian[1]]);
                used[zuijindian[1]] = 0;
                num--;
                jishu++;
            }
        }
        }


    }

    if (num != 0)
    {
        vector<int> zuijindian = xunzuijin(xy, used, xy_good[jishu - 1]);

        if (zuijindian.size() == 0) return;
        if (zuijindian.size() == 1)
        {

            xy_good.push_back(xy[zuijindian[0]]);
            used[zuijindian[0]] = 0;
            num--;
            jishu++;
        }
        else
        {
        if (abs(diantoxian(xy[zuijindian[0]], xy_good[0], { 0,xy_good[0].second }) - diantoxian(xy[zuijindian[1]], xy_good[0], { 0,xy_good[0].second })) < 0.1)
        {
            xy_good.push_back(xy[zuijindian[0]]);
            used[zuijindian[0]] = 0;
            num--;
            jishu++;
        }
        else
        {
            if (diantoxian(xy[zuijindian[0]], xy_good[jishu - 1], xy_good[jishu - 2]) < diantoxian(xy[zuijindian[1]], xy_good[jishu - 1], xy_good[jishu - 2]))
            {

                xy_good.push_back(xy[zuijindian[0]]);
                used[zuijindian[0]] = 0;
                num--;
                jishu++;
            }
            else
            {

                xy_good.push_back(xy[zuijindian[1]]);
                used[zuijindian[1]] = 0;
                num--;
                jishu++;
            }
        }
        }



    }


    if (pandian(xy_good) == 0)
    {
        xy_good.pop_back();
        return;
    }
    zhuitongluxian(xy, xy_good, used, num, jishu);
}

class TSPGeneticAlgorithm {
public:
    const int POP_SIZE = 30;          // 种群大小
    const int GENERATIONS = 500;      // 迭代次数
    const double MUTATION_RATE = 0.1;   // 变异率

    vector<pair<double, double>> cities;
    int numCities;
    int startCity;    // 固定第一个城市
    int secondCity;   // 固定第二个城市

    typedef vector<int> Individual;

    // 构造函数，要求输入的城市序列中第一个和第二个城市分别为起始点和第二座城市
    TSPGeneticAlgorithm(const vector<pair<double, double>>& lujingdian) {
        cities = lujingdian;
        numCities = lujingdian.size();
        startCity = 0;    // 第一个点
        secondCity = 1;   // 第二个点
    }

    // 计算两个城市的欧几里得距离
    double euclideanDistance(int i, int j) {
        double dx = cities[i].first - cities[j].first;
        double dy = cities[i].second - cities[j].second;
        return sqrt(dx * dx + dy * dy);
    }

    // 计算一个个体的路径总距离（路径：起始点 -> 第二城市 -> 后续城市 -> 返回起始点）
    double calculateDistance(const Individual& individual) {
        double totalDist = 0.0;
        // 路径从起始点到第二城市
        totalDist += euclideanDistance(individual[0], individual[1]);
        // 后续城市间的距离
        for (int i = 1; i < numCities - 1; ++i) {
            totalDist += euclideanDistance(individual[i], individual[i + 1]);
        }
        // 最后返回起始点
        totalDist += euclideanDistance(individual[numCities - 1], individual[0]);
        return totalDist;
    }

    // 初始化种群，个体中第 0 和第 1 个城市分别固定为 startCity 和 secondCity
    vector<Individual> initializePopulation() {
        vector<Individual> population;
        // 剩余可变的城市序列（排除固定的两个城市）
        vector<int> otherCities;
        for (int i = 0; i < numCities; ++i) {
            if (i != startCity && i != secondCity)
                otherCities.push_back(i);
        }
        for (int i = 0; i < POP_SIZE; ++i) {
            Individual individual;
            // 固定起始点和第二城市
            individual.push_back(startCity);
            individual.push_back(secondCity);
            vector<int> perm = otherCities;
            random_shuffle(perm.begin(), perm.end());
            individual.insert(individual.end(), perm.begin(), perm.end());
            population.push_back(individual);
        }
        return population;
    }

    // 选择操作：采用轮盘赌选择，适应度与路径总距离的倒数成正比
    Individual selectParent(const vector<Individual>& population) {
        double totalFitness = 0.0;
        vector<double> cumulativeFitness;
        for (const auto& individual : population) {
            double fit = 1.0 / calculateDistance(individual);
            totalFitness += fit;
            cumulativeFitness.push_back(totalFitness);
        }
        double r = ((double)rand() / RAND_MAX) * totalFitness;
        for (int i = 0; i < POP_SIZE; ++i) {
            if (r < cumulativeFitness[i])
                return population[i];
        }
        return population[POP_SIZE - 1];
    }

    // 交叉操作：单点交叉，仅对下标 2 到 numCities-1 部分进行，保证前两个城市不变
    Individual crossover(const Individual& parent1, const Individual& parent2) {
        Individual child(numCities, -1);
        // 固定前两个城市
        child[0] = startCity;
        child[1] = secondCity;
        // 交叉只在可变部分进行
        int crossoverPoint = 2 + rand() % (numCities - 2);
        for (int i = 2; i < crossoverPoint; ++i) {
            child[i] = parent1[i];
        }
        int index = crossoverPoint;
        for (int i = 2; i < numCities; ++i) {
            if (find(child.begin() + 2, child.end(), parent2[i]) == child.end()) {
                child[index++] = parent2[i];
            }
        }
        return child;
    }

    // 变异操作：随机交换个体中两个城市的位置（仅在下标 2...numCities-1之间交换）
    void mutate(Individual& individual) {
        if (((double)rand() / RAND_MAX) < MUTATION_RATE) {
            int i = 2 + rand() % (numCities - 2);
            int j = 2 + rand() % (numCities - 2);
            swap(individual[i], individual[j]);
        }
    }

    // 遗传算法主循环（引入精英策略）
    vector<pair<double, double>> solveTSP() {
        vector<Individual> population = initializePopulation();
        for (int generation = 0; generation < GENERATIONS; ++generation) {
            vector<Individual> newPopulation;
            int eliteCount = max(1, POP_SIZE / 5);

            // 按路径距离从小到大排序，距离越小越优
            sort(population.begin(), population.end(), [this](const Individual& a, const Individual& b) {
                return calculateDistance(a) < calculateDistance(b);
                });

            // 将精英个体直接复制到新种群中
            for (int i = 0; i < eliteCount; ++i) {
                newPopulation.push_back(population[i]);
            }

            // 生成剩余的新个体
            while (newPopulation.size() < population.size()) {
                Individual parent1 = selectParent(population);
                Individual parent2 = selectParent(population);
                Individual child = crossover(parent1, parent2);
                mutate(child);
                newPopulation.push_back(child);
            }

            population = newPopulation;
        }

        // 在最后一代中找到最佳个体
        Individual bestIndividual = population[0];
        double bestDistance = calculateDistance(bestIndividual);
        for (const auto& individual : population) {
            double d = calculateDistance(individual);
            if (d < bestDistance) {
                bestIndividual = individual;
                bestDistance = d;
            }
        }

        // 构造输出：直接根据最佳个体顺序返回城市坐标，
        // 此时 bestIndividual 的第 0 和第 1 个元素必定是输入的第一个和第二个城市
        vector<pair<double, double>> lujingdian2;
        for (int idx : bestIndividual) {
            lujingdian2.push_back(cities[idx]);
        }
        return lujingdian2;
    }
};


//路径点选取
vector<std::pair<double, double>> lujingdianxuanqu(vector<std::pair<double, double>> lujing_dian)
{
    int numzheng = 0;
    int numfu = 0;
    for (int i = 0; i < lujing_dian.size(); i++)
    {
        if (lujing_dian[i].second > 0)
        {
            numzheng++;
        }
        else
        {
            numfu++;
        }
    }

    int qishi = 0;
    if (numzheng > 75)
    {
        for (int i = 0; i < lujing_dian.size(); i++)
        {
            if (lujing_dian[i].second > 0)
            {
                qishi = i;
                break;
            }
        }
    }

    if (numfu > 75)
    {
        for (int i = 0; i < lujing_dian.size(); i++)
        {
            if (lujing_dian[i].second < 0)
            {
                qishi = i;
                break;
            }
        }
    }

    //cout << "qishi" << qishi << endl;


    //根据起始点确认路径点，四个路径点
    int duan = (lujing_dian.size() - qishi) / 5;

    vector<std::pair<double, double>> result;

    result.push_back({ 0,0 });
    for (int i = 0; i < 4; i++)
    {
        result.push_back(lujing_dian[qishi + i * duan]);
    }

    return result;
}

void weizhizhuitong(vector<std::pair<double, double>>& xy, vector<bool>& used, vector<std::pair<double, double>>& l_xy_good, vector<std::pair<double, double>>& r_xy_good, bool l, bool r, int num)
{
    if (!(l || r) || num <= 2)
    {
        return;
    }

    if (l && num >= 2)
    {
        vector<int> zuijindianl = xunzuijin(xy, used, l_xy_good.back());
        if (l_xy_good.size() <= 1)
        {
            if (abs(diantoxian(xy[zuijindianl[0]], l_xy_good.back(), { 0,l_xy_good.back().second }) - diantoxian(xy[zuijindianl[1]], l_xy_good.back(), { 0,l_xy_good.back().second })) < 0.1)
            {

                l_xy_good.push_back(xy[zuijindianl[0]]);
                used[zuijindianl[0]] = 0;
                num--;

                if (pandian(l_xy_good) == 0)
                {
                    l_xy_good.pop_back();
                    l = 0;
                    num++;
                    used[zuijindianl[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), { 0,l_xy_good.back().second }) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), { 0,l_xy_good.back().second }))
                {

                    l_xy_good.push_back(xy[zuijindianl[0]]);
                    used[zuijindianl[0]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[0]] = 1;
                    }
                }
                else
                {

                    l_xy_good.push_back(xy[zuijindianl[1]]);
                    used[zuijindianl[1]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[1]] = 1;
                    }
                }
            }


        }
        else
        {
            if (abs(diantoxian(xy[zuijindianl[0]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]) - diantoxian(xy[zuijindianl[1]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2])) < 0.1)
            {
                l_xy_good.push_back(xy[zuijindianl[0]]);
                used[zuijindianl[0]] = 0;
                num--;

                if (pandian(l_xy_good) == 0)
                {
                    l_xy_good.pop_back();
                    l = 0;
                    num++;
                    used[zuijindianl[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]))
                {
                    l_xy_good.push_back(xy[zuijindianl[0]]);
                    used[zuijindianl[0]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[0]] = 1;
                    }
                }
                else
                {
                    l_xy_good.push_back(xy[zuijindianl[1]]);
                    used[zuijindianl[1]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[1]] = 1;
                    }
                }
            }


        }

    }

    if (r && num >= 2)
    {
        vector<int> zuijindianr = xunzuijin(xy, used, r_xy_good.back());
        if (r_xy_good.size() <= 1)
        {
            if (abs(diantoxian(xy[zuijindianr[0]], r_xy_good.back(), { 0,r_xy_good.back().second }) - diantoxian(xy[zuijindianr[1]], r_xy_good.back(), { 0,r_xy_good.back().second })) < 0.1)
            {
                r_xy_good.push_back(xy[zuijindianr[0]]);
                used[zuijindianr[0]] = 0;
                num--;

                if (pandian(r_xy_good) == 0)
                {
                    r_xy_good.pop_back();
                    r = 0;
                    num++;
                    used[zuijindianr[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), { 0,r_xy_good.back().second }) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), { 0,r_xy_good.back().second }))
                {
                    r_xy_good.push_back(xy[zuijindianr[0]]);
                    used[zuijindianr[0]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[0]] = 1;
                    }
                }
                else
                {
                    r_xy_good.push_back(xy[zuijindianr[1]]);
                    used[zuijindianr[1]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[1]] = 1;
                    }
                }
            }
        }



        else
        {
            if (abs(diantoxian(xy[zuijindianr[0]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]) - diantoxian(xy[zuijindianr[1]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2])) < 0.1)
            {
                r_xy_good.push_back(xy[zuijindianr[0]]);
                used[zuijindianr[0]] = 0;
                num--;

                if (pandian(r_xy_good) == 0)
                {
                    r_xy_good.pop_back();
                    r = 0;
                    num++;
                    used[zuijindianr[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]))
                {
                    r_xy_good.push_back(xy[zuijindianr[0]]);
                    used[zuijindianr[0]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[0]] = 1;
                    }
                }
                else
                {
                    r_xy_good.push_back(xy[zuijindianr[1]]);
                    used[zuijindianr[1]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[1]] = 1;
                    }
                }
            }


        }

        weizhizhuitong(xy, used, l_xy_good, r_xy_good, l, r, num);

    }

}


void weizhizhuitong_jingxiang(vector<std::pair<double, double>>& xy, vector<bool>& used, vector<std::pair<double, double>>& l_xy_good, vector<std::pair<double, double>>& r_xy_good, bool l, bool r, int num)
{
    if (!(l || r) || num <= 2)
    {
        return;
    }



    if (r && num >= 2)
    {
        vector<int> zuijindianr = xunzuijin(xy, used, r_xy_good.back());
        if (r_xy_good.size() <= 1)
        {
            if (abs(diantoxian(xy[zuijindianr[0]], r_xy_good.back(), { 0,r_xy_good.back().second }) - diantoxian(xy[zuijindianr[1]], r_xy_good.back(), { 0,r_xy_good.back().second })) < 0.1)
            {
                r_xy_good.push_back(xy[zuijindianr[0]]);
                used[zuijindianr[0]] = 0;
                num--;

                if (pandian(r_xy_good) == 0)
                {
                    r_xy_good.pop_back();
                    r = 0;
                    num++;
                    used[zuijindianr[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), { 0,r_xy_good.back().second }) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), { 0,r_xy_good.back().second }))
                {
                    r_xy_good.push_back(xy[zuijindianr[0]]);
                    used[zuijindianr[0]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[0]] = 1;
                    }
                }
                else
                {
                    r_xy_good.push_back(xy[zuijindianr[1]]);
                    used[zuijindianr[1]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[1]] = 1;
                    }
                }
            }
        }



        else
        {
            if (abs(diantoxian(xy[zuijindianr[0]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]) - diantoxian(xy[zuijindianr[1]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2])) < 0.1)
            {
                r_xy_good.push_back(xy[zuijindianr[0]]);
                used[zuijindianr[0]] = 0;
                num--;

                if (pandian(r_xy_good) == 0)
                {
                    r_xy_good.pop_back();
                    r = 0;
                    num++;
                    used[zuijindianr[0]] = 1;
                }
            }
            else
            {
                if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]))
                {
                    r_xy_good.push_back(xy[zuijindianr[0]]);
                    used[zuijindianr[0]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[0]] = 1;
                    }
                }
                else
                {
                    r_xy_good.push_back(xy[zuijindianr[1]]);
                    used[zuijindianr[1]] = 0;
                    num--;

                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        r = 0;
                        num++;
                        used[zuijindianr[1]] = 1;
                    }
                }
            }


        }

        if (l && num >= 2)
        {
            vector<int> zuijindianl = xunzuijin(xy, used, l_xy_good.back());
            if (l_xy_good.size() <= 1)
            {
                if (abs(diantoxian(xy[zuijindianl[0]], l_xy_good.back(), { 0,l_xy_good.back().second }) - diantoxian(xy[zuijindianl[1]], l_xy_good.back(), { 0,l_xy_good.back().second })) < 0.1)
                {

                    l_xy_good.push_back(xy[zuijindianl[0]]);
                    used[zuijindianl[0]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[0]] = 1;
                    }
                }
                else
                {
                    if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), { 0,l_xy_good.back().second }) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), { 0,l_xy_good.back().second }))
                    {

                        l_xy_good.push_back(xy[zuijindianl[0]]);
                        used[zuijindianl[0]] = 0;
                        num--;

                        if (pandian(l_xy_good) == 0)
                        {
                            l_xy_good.pop_back();
                            l = 0;
                            num++;
                            used[zuijindianl[0]] = 1;
                        }
                    }
                    else
                    {

                        l_xy_good.push_back(xy[zuijindianl[1]]);
                        used[zuijindianl[1]] = 0;
                        num--;

                        if (pandian(l_xy_good) == 0)
                        {
                            l_xy_good.pop_back();
                            l = 0;
                            num++;
                            used[zuijindianl[1]] = 1;
                        }
                    }
                }


            }
            else
            {
                if (abs(diantoxian(xy[zuijindianl[0]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]) - diantoxian(xy[zuijindianl[1]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2])) < 0.1)
                {
                    l_xy_good.push_back(xy[zuijindianl[0]]);
                    used[zuijindianl[0]] = 0;
                    num--;

                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        l = 0;
                        num++;
                        used[zuijindianl[0]] = 1;
                    }
                }
                else
                {
                    if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]))
                    {
                        l_xy_good.push_back(xy[zuijindianl[0]]);
                        used[zuijindianl[0]] = 0;
                        num--;

                        if (pandian(l_xy_good) == 0)
                        {
                            l_xy_good.pop_back();
                            l = 0;
                            num++;
                            used[zuijindianl[0]] = 1;
                        }
                    }
                    else
                    {
                        l_xy_good.push_back(xy[zuijindianl[1]]);
                        used[zuijindianl[1]] = 0;
                        num--;

                        if (pandian(l_xy_good) == 0)
                        {
                            l_xy_good.pop_back();
                            l = 0;
                            num++;
                            used[zuijindianl[1]] = 1;
                        }
                    }
                }


            }

        }
        weizhizhuitong(xy, used, l_xy_good, r_xy_good, l, r, num);

    }

}
//if (!(l || r) || num <= 2)
//{
//    return;
//}
//
//if (l && num >= 2)
//{
//    vector<int> zuijindianl = xunzuijin(xy, used, l_xy_good.back());
//    if (l_xy_good.size() <= 1)
//    {
//        if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), { 0,l_xy_good.back().second }) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), { 0,l_xy_good.back().second }))
//        {
//            l_xy_good.push_back(xy[zuijindianl[0]]);
//            used[zuijindianl[0]] = 0;
//            num--;
//
//            if (pandian(l_xy_good) == 0)
//            {
//                l_xy_good.pop_back();
//                l = 0;
//                num++;
//                used[zuijindianl[0]] = 1;
//            }
//        }
//        else
//        {
//            l_xy_good.push_back(xy[zuijindianl[1]]);
//            used[zuijindianl[1]] = 0;
//            num--;
//
//            if (pandian(l_xy_good) == 0)
//            {
//                l_xy_good.pop_back();
//                l = 0;
//                num++;
//                used[zuijindianl[1]] = 1;
//            }
//        }
//
//    }
//    else
//    {
//        if (diantoxian(xy[zuijindianl[0]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]) < diantoxian(xy[zuijindianl[1]], l_xy_good.back(), l_xy_good[l_xy_good.size() - 2]))
//        {
//            l_xy_good.push_back(xy[zuijindianl[0]]);
//            used[zuijindianl[0]] = 0;
//            num--;
//
//            if (pandian(l_xy_good) == 0)
//            {
//                l_xy_good.pop_back();
//                l = 0;
//                num++;
//                used[zuijindianl[0]] = 1;
//            }
//        }
//        else
//        {
//            l_xy_good.push_back(xy[zuijindianl[1]]);
//            used[zuijindianl[1]] = 0;
//            num--;
//
//            if (pandian(l_xy_good) == 0)
//            {
//                l_xy_good.pop_back();
//                l = 0;
//                num++;
//                used[zuijindianl[1]] = 1;
//            }
//        }
//
//    }
//
//}
//
//if (r && num >= 2)
//{
//    vector<int> zuijindianr = xunzuijin(xy, used, r_xy_good.back());
//    if (r_xy_good.size() <= 1)
//    {
//        if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), { 0,r_xy_good.back().second }) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), { 0,r_xy_good.back().second }))
//        {
//            r_xy_good.push_back(xy[zuijindianr[0]]);
//            used[zuijindianr[0]] = 0;
//            num--;
//
//            if (pandian(r_xy_good) == 0)
//            {
//                r_xy_good.pop_back();
//                r = 0;
//                num++;
//                used[zuijindianr[0]] = 1;
//            }
//        }
//        else
//        {
//            r_xy_good.push_back(xy[zuijindianr[1]]);
//            used[zuijindianr[1]] = 0;
//            num--;
//
//            if (pandian(r_xy_good) == 0)
//            {
//                r_xy_good.pop_back();
//                r = 0;
//                num++;
//                used[zuijindianr[1]] = 1;
//            }
//        }
//
//    }
//    else
//    {
//        if (diantoxian(xy[zuijindianr[0]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]) < diantoxian(xy[zuijindianr[1]], r_xy_good.back(), r_xy_good[r_xy_good.size() - 2]))
//        {
//            r_xy_good.push_back(xy[zuijindianr[0]]);
//            used[zuijindianr[0]] = 0;
//            num--;
//
//            if (pandian(r_xy_good) == 0)
//            {
//                r_xy_good.pop_back();
//                r = 0;
//                num++;
//                used[zuijindianr[0]] = 1;
//            }
//        }
//        else
//        {
//            r_xy_good.push_back(xy[zuijindianr[1]]);
//            used[zuijindianr[1]] = 0;
//            num--;
//
//            if (pandian(r_xy_good) == 0)
//            {
//                r_xy_good.pop_back();
//                r = 0;
//                num++;
//                used[zuijindianr[1]] = 1;
//            }
//        }
//
//    }
//
//    weizhizhuitong(xy, used, l_xy_good, r_xy_good, l, r, num);
//
//}




class zhuitong {
public:
    std::vector<std::pair<double, double>> l_xy, r_xy;
    std::vector<std::pair<double, double>> l_xy_good, r_xy_good;
    std::vector<std::pair<double, double>> lujing_dian;
    std::vector<std::pair<double, double>> lujing_dian2;
    std::vector<std::pair<double, double>> xy;
    void shaixuan() {
        // 筛选出前方的锁桶
        std::vector<std::pair<double, double>> filtered_l_xy;
        for (const auto& cone : l_xy) {
            if (cone.first > 0.2) {
                filtered_l_xy.push_back(cone);
            }
        }
        l_xy = std::move(filtered_l_xy);

        std::vector<std::pair<double, double>> filtered_r_xy;
        for (const auto& cone : r_xy) {
            if (cone.first > 0.2) {
                filtered_r_xy.push_back(cone);
            }
        }
        r_xy = std::move(filtered_r_xy);

        //// 输出筛选后的结果
        //std::cout << "筛选后的前方左锁桶：" << std::endl;
        //for (const auto& cone : l_xy) {
        //    std::cout << "x: " << cone.first << ", y: " << cone.second << std::endl;
        //}
        //std::cout << "筛选后的前方右锁桶：" << std::endl;
        //for (const auto& cone : r_xy) {
        //    std::cout << "x: " << cone.first << ", y: " << cone.second << std::endl;
        //}
    }

    void youxiaozhuitong() {
        //排序
        sortPointsByDistance(l_xy, { 0,0 });
        sortPointsByDistance(r_xy, { 0,0 });

        //
        vector<bool> usedl(l_xy.size(), 1);
        vector<bool> usedr(r_xy.size(), 1);

        l_xy_good.clear();
        r_xy_good.clear();

        zhuitongluxian(l_xy, l_xy_good, usedl, l_xy.size(), 0);
        zhuitongluxian(r_xy, r_xy_good, usedr, r_xy.size(), 0);





        // // 输出筛选后的结果
        // std::cout << "筛选后的前方左有效锁桶：" << std::endl;
        // for (const auto& cone : l_xy_good) {
        //     std::cout << "x: " << cone.first << ", y: " << cone.second << std::endl;
        // }
        // std::cout << "筛选后的前方有效右锁桶：" << std::endl;
        // for (const auto& cone : r_xy_good) {
        //     std::cout << "x: " << cone.first << ", y: " << cone.second << std::endl;
        // }





    }

    void youxiaozhuitong_tsp()
    {
        //排序
        sortPointsByDistance(l_xy, { 0,0 });
        sortPointsByDistance(r_xy, { 0,0 });



        if (r_xy.size() <= 1)
        {
            r_xy_good = r_xy;
        }
        else
        {
            vector<bool> used(r_xy.size(), 1);
            used[0] = 0;
            vector<int> zuijindian = xunzuijin(r_xy, used, r_xy[0]);

            if (diantoxian(r_xy[zuijindian[0]], r_xy[0], { 0,r_xy[0].second }) < diantoxian(r_xy[zuijindian[1]], r_xy[0], { 0,r_xy[0].second }))
            {
                swap(r_xy[1], r_xy[zuijindian[0]]);
            }
            else
            {
                swap(r_xy[1], r_xy[zuijindian[1]]);
            }




            //图论遗传算法
            TSPGeneticAlgorithm tspSolver_r(r_xy);
            vector<pair<double, double>> r_xy_good_tmp = tspSolver_r.solveTSP();

            for (int i = 0; i < r_xy_good_tmp.size(); i++)
            {
                //std::cout <<"l"<<i << ":x: " << r_xy_good_tmp[i].first << ", y: " << r_xy_good_tmp[i].second << std::endl;
                r_xy_good.push_back(r_xy_good_tmp[i]);
                if (i > 1)
                {
                    if (pandian(r_xy_good) == 0)
                    {
                        r_xy_good.pop_back();
                        break;
                    }
                }
            }

        }

        if (l_xy.size() <= 1)
        {
            l_xy_good = l_xy;
        }
        else
        {
            vector<bool> used(l_xy.size(), 1);
            used[0] = 0;
            vector<int> zuijindian = xunzuijin(l_xy, used, l_xy[0]);

            if (diantoxian(l_xy[zuijindian[0]], l_xy[0], { 0,l_xy[0].second }) < diantoxian(l_xy[zuijindian[1]], l_xy[0], { 0,l_xy[0].second }))
            {
                swap(l_xy[1], l_xy[zuijindian[0]]);
            }
            else
            {
                swap(l_xy[1], l_xy[zuijindian[1]]);
            }

            TSPGeneticAlgorithm tspSolver_l(l_xy);
            vector<pair<double, double>> l_xy_good_tmp = tspSolver_l.solveTSP();

            for (int i = 0; i < l_xy_good_tmp.size(); i++)
            {
                //std::cout <<"r"<<i << ":x: " << l_xy_good_tmp[i].first << ", y: " << l_xy_good_tmp[i].second << std::endl;
                l_xy_good.push_back(l_xy_good_tmp[i]);
                if (i > 1)
                {
                    if (pandian(l_xy_good) == 0)
                    {
                        l_xy_good.pop_back();
                        break;
                    }
                }
            }
        }


    }


    void youxiaozhuitong_weizhi()
    {

        // 筛选出前方的锁桶
        std::vector<std::pair<double, double>> filtered_xy;
        for (const auto& cone : xy) {
            if (cone.first > 0.2) {
                filtered_xy.push_back(cone);
            }
        }
        xy = std::move(filtered_xy);

        vector<bool> used(xy.size(), 1);
        std::pair<double, double> yuandian = { 0,0 };

        vector<int> zuijindian = xunzuijin(xy, used, yuandian);

        //左右第一个点

        if (xy[zuijindian[0]].second * xy[zuijindian[1]].second <= 0)
        {

            l_xy_good.push_back(xy[zuijindian[0]]);
            r_xy_good.push_back(xy[zuijindian[1]]);
            used[zuijindian[0]] = 0;
            used[zuijindian[1]] = 0;
        }
        else
        {
            if (distance(xy[zuijindian[0]], { 0,0 }) < distance(xy[zuijindian[1]], { 0,0 }))
            {
                used[zuijindian[1]] = 0;
            }
            else
            {
                used[zuijindian[0]] = 0;
            }

            zuijindian = xunzuijin(xy, used, yuandian);

            if (xy[zuijindian[0]].second * xy[zuijindian[1]].second <= 0)
            {
                l_xy_good.push_back(xy[zuijindian[0]]);
                r_xy_good.push_back(xy[zuijindian[1]]);
                used[zuijindian[0]] = 0;
                used[zuijindian[1]] = 0;
            }
            else
            {
                if (distance(xy[zuijindian[0]], { 0,0 }) < distance(xy[zuijindian[1]], { 0,0 }))
                {
                    used[zuijindian[1]] = 0;
                }
                else
                {
                    used[zuijindian[0]] = 0;
                }

                zuijindian = xunzuijin(xy, used, yuandian);
                l_xy_good.push_back(xy[zuijindian[0]]);
                r_xy_good.push_back(xy[zuijindian[1]]);
                used[zuijindian[0]] = 0;
                used[zuijindian[1]] = 0;
            }
        }

        int l = 1;
        int r = 1;
        int num = xy.size() - 2;

        vector< std::pair<double, double>> tmp_l_xy_good = l_xy_good;
        vector< std::pair<double, double>> tmp_r_xy_good = r_xy_good;
        int tmpl = 1;
        int tmpr = 1;
        int tmpnum = xy.size() - 2;
        vector<bool> tmpused = used;

        weizhizhuitong(xy, used, l_xy_good, r_xy_good, l, r, num);

        weizhizhuitong_jingxiang(xy, tmpused, tmp_l_xy_good, tmp_r_xy_good, tmpl, tmpr, tmpnum);

        if (tmp_r_xy_good.size() + tmp_l_xy_good.size() - l_xy_good.size() - r_xy_good.size() > 0)
        {
            l_xy_good = tmp_l_xy_good;
            r_xy_good = tmp_r_xy_good;
        }


      //  cout<<"lgood"<<l_xy_good.size()<<endl;
        //cout<<"rgood"<<r_xy_good.size()<<endl;
    }

    void lujingdian() {
        lujing_dian2.push_back({ 0,0 });
       // std::cout << "lgoodsize" << l_xy_good.size() << endl;
       // std::cout << "rgoodsize" << r_xy_good.size() << endl;

        if (l_xy_good.size() >= 4 && r_xy_good.size() >= 4)
        {
            int numPoints = 100;
            vector<std::pair<double, double>> controlPointsl = l_xy_good;
            vector<std::pair<double, double>> controlPointsr = r_xy_good;



            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 3) * controlPointsl[0].first +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].first +
                    3 * (1 - t) * t * t * controlPointsl[2].first +
                    t * t * t * controlPointsl[3].first;

                double yl = std::pow(1 - t, 3) * controlPointsl[0].second +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].second +
                    3 * (1 - t) * t * t * controlPointsl[2].second +
                    t * t * t * controlPointsl[3].second;

                double xr = std::pow(1 - t, 3) * controlPointsr[0].first +
                    3 * std::pow(1 - t, 2) * t * controlPointsr[1].first +
                    3 * (1 - t) * t * t * controlPointsr[2].first +
                    t * t * t * controlPointsr[3].first;

                double yr = std::pow(1 - t, 3) * controlPointsr[0].second +
                    3 * std::pow(1 - t, 2) * t * controlPointsr[1].second +
                    3 * (1 - t) * t * t * controlPointsr[2].second +
                    t * t * t * controlPointsr[3].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }

        if ((l_xy_good.size() >= 4 && r_xy_good.size() == 3) || (l_xy_good.size() == 3 && r_xy_good.size() >= 4))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 3) * controlPointsl[0].first +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].first +
                    3 * (1 - t) * t * t * controlPointsl[2].first +
                    t * t * t * controlPointsl[3].first;

                double yl = std::pow(1 - t, 3) * controlPointsl[0].second +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].second +
                    3 * (1 - t) * t * t * controlPointsl[2].second +
                    t * t * t * controlPointsl[3].second;

                double xr = std::pow(1 - t, 2) * controlPointsr[0].first +
                    2 * (1 - t) * t * controlPointsr[1].first +
                    std::pow(t, 2) * controlPointsr[2].first;

                double yr = std::pow(1 - t, 2) * controlPointsr[0].second +
                    2 * (1 - t) * t * controlPointsr[1].second +
                    std::pow(t, 2) * controlPointsr[2].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }

        if ((l_xy_good.size() == 3 && r_xy_good.size() == 3))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 2) * controlPointsl[0].first +
                    2 * (1 - t) * t * controlPointsl[1].first +
                    std::pow(t, 2) * controlPointsl[2].first;

                double yl = std::pow(1 - t, 2) * controlPointsl[0].second +
                    2 * (1 - t) * t * controlPointsl[1].second +
                    std::pow(t, 2) * controlPointsl[2].second;

                double xr = std::pow(1 - t, 2) * controlPointsr[0].first +
                    2 * (1 - t) * t * controlPointsr[1].first +
                    std::pow(t, 2) * controlPointsr[2].first;

                double yr = std::pow(1 - t, 2) * controlPointsr[0].second +
                    2 * (1 - t) * t * controlPointsr[1].second +
                    std::pow(t, 2) * controlPointsr[2].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }


        if ((l_xy_good.size() >= 4 && r_xy_good.size() == 2) || (l_xy_good.size() == 2 && r_xy_good.size() >= 4))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 3) * controlPointsl[0].first +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].first +
                    3 * (1 - t) * t * t * controlPointsl[2].first +
                    t * t * t * controlPointsl[3].first;

                double yl = std::pow(1 - t, 3) * controlPointsl[0].second +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].second +
                    3 * (1 - t) * t * t * controlPointsl[2].second +
                    t * t * t * controlPointsl[3].second;

                double xr = (1 - t) * controlPointsr[0].first +
                    t * controlPointsr[1].first;


                double yr = (1 - t) * controlPointsr[0].second +
                    t * controlPointsr[1].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }

        if ((l_xy_good.size() == 2 && r_xy_good.size() == 2))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = (1 - t) * controlPointsl[0].first +
                    t * controlPointsl[1].first;


                double yl = (1 - t) * controlPointsl[0].second +
                    t * controlPointsl[1].second;

                double xr = (1 - t) * controlPointsr[0].first +
                    t * controlPointsr[1].first;


                double yr = (1 - t) * controlPointsr[0].second +
                    t * controlPointsr[1].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }

        if ((l_xy_good.size() == 3 && r_xy_good.size() == 1) || (l_xy_good.size() == 1 && r_xy_good.size() == 3))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 2) * controlPointsl[0].first +
                    2 * (1 - t) * t * controlPointsl[1].first +
                    std::pow(t, 2) * controlPointsl[2].first;

                double yl = std::pow(1 - t, 2) * controlPointsl[0].second +
                    2 * (1 - t) * t * controlPointsl[1].second +
                    std::pow(t, 2) * controlPointsl[2].second;

                double xr = controlPointsr[0].first;



                double yr = controlPointsr[0].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }

        if ((l_xy_good.size() == 2 && r_xy_good.size() == 1) || (l_xy_good.size() == 1 && r_xy_good.size() == 2))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = (1 - t) * controlPointsl[0].first +
                    t * controlPointsl[1].first;


                double yl = (1 - t) * controlPointsl[0].second +
                    t * controlPointsl[1].second;

                double xr = controlPointsr[0].first;



                double yr = controlPointsr[0].second;

                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }
        if ((l_xy_good.size() >= 4 && r_xy_good.size() == 1) || (l_xy_good.size() == 1 && r_xy_good.size() >= 4))
        {
            int numPoints = 100;

            vector<std::pair<double, double>> controlPointsl;
            vector<std::pair<double, double>> controlPointsr;

            if (l_xy_good.size() > r_xy_good.size())
            {
                controlPointsl = l_xy_good;
                controlPointsr = r_xy_good;
            }
            else
            {
                controlPointsl = r_xy_good;
                controlPointsr = l_xy_good;
            }




            for (int j = 0; j <= numPoints; ++j) {
                double t = static_cast<double>(j) / numPoints; // 参数 t

                double xl = std::pow(1 - t, 3) * controlPointsl[0].first +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].first +
                    3 * (1 - t) * t * t * controlPointsl[2].first +
                    t * t * t * controlPointsl[3].first;

                double yl = std::pow(1 - t, 3) * controlPointsl[0].second +
                    3 * std::pow(1 - t, 2) * t * controlPointsl[1].second +
                    3 * (1 - t) * t * t * controlPointsl[2].second +
                    t * t * t * controlPointsl[3].second;

                double xr = controlPointsr[0].first;



                double yr = controlPointsr[0].second;


                lujing_dian.emplace_back((xl + xr) / 2, (yr + yl) / 2);



            }
        }
        lujing_dian2 = lujingdianxuanqu(lujing_dian);

        //// 输出筛选后的结果
        //std::cout << "lujing2：" << std::endl;
        //for (const auto& cone : lujing_dian2) {
        //    std::cout << "x: " << cone.first << ", y: " << cone.second << std::endl;
        //}


    }
};

class tiaoyang
{
public:
    //坐标点记录
    vector<std::pair<double, double>> dian;
    int num; //num是坐标点数量
    vector<double> a;
    vector<double> b;
    vector<double> c;
    vector<double> d;

    tiaoyang(vector<std::pair<double, double>>& lujing2)
    {
        dian = lujing2;
        num = dian.size();

        // Step 1: 计算区间长度 h 和斜率差分
        vector<double> h(num - 1), alpha(num - 1);
        for (int i = 0; i < num - 1; ++i) {
            h[i] = dian[i + 1].first - dian[i].first;
            alpha[i] = (dian[i + 1].second - dian[i].second) / h[i];
        }

        // Step 2: 构造三对角矩阵求解二阶导数 m
        vector<double> l(num), mu(num), z(num), m(num, 0);
        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;

        for (int i = 1; i < num - 1; ++i) {
            l[i] = 2 * (dian[i + 1].first - dian[i - 1].first) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - alpha[i - 1] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[num - 1] = 1;
        z[num - 1] = 0;

        for (int j = num - 2; j >= 0; --j) {
            m[j] = z[j] - mu[j] * m[j + 1];
        }

        // Step 3: 计算 a, b, c, d
        a.resize(num - 1);
        b.resize(num - 1);
        c.resize(num - 1);
        d.resize(num - 1);

        for (int i = 0; i < num - 1; ++i) {
            a[i] = dian[i].second;
            b[i] = alpha[i] - h[i] * (2 * m[i] + m[i + 1]) / 6.0;
            c[i] = m[i] / 2.0;
            d[i] = (m[i + 1] - m[i]) / (6.0 * h[i]);
        }

        // 输出结果
        for (int i = 0; i < num - 1; ++i) {
            //std::cout << "Segment " << i + 1 << ": " << endl;
            //std::cout << "a: " << a[i] << ", b: " << b[i] << ", c: " << c[i] << ", d: " << d[i] << endl;
        }
    }

    double zhi(double x)
    {
        int duan;
        for (int i = 0; i < num; i++)
        {
            if (x >= dian[i].first && x < dian[i + 1].first) duan = i;
        }

        return a[duan] + b[duan] * (x - dian[duan].first) + c[duan] * pow(x - dian[duan].first, 2) + d[duan] * pow(x - dian[duan].first, 3);
    }

    MX xielv(MX x)
    {
        /*int duan;
        for (int i = 0; i < num; i++)
        {
            if (MX >= dian[i].first && x < dian[i + 1].first) duan = i;
        }

        return  b[duan] + c[duan] * pow(x - dian[duan].first, 1) + d[duan] * pow(x - dian[duan].first, 2);*/

        MX dx = 0.0;
        MX result = 0.0;

        for (int i = 0; i < num; i++)
        {

            MX condition = if_else((x >= MX(dian[i].first) && x < MX(dian[i + 1].first)), 1.0, 0.0);
            dx = x - dian[i].first;
            result += condition * (b[i] + c[i] * pow(dx, 1) + d[i] * pow(dx, 2));
        }
        return result;
    }

    MX zhicasadi(MX x)
    {
        MX dx = 0.0;
        MX result = 0.0;

        for (int i = 0; i < num; i++)
        {

            MX condition = if_else((x >= MX(dian[i].first) && x < MX(dian[i + 1].first)), 1.0, 0.0);
            dx = x - dian[i].first;
            result += condition * (a[i] + b[i] * pow(dx, 1) + c[i] * pow(dx, 2) + d[i] * pow(dx, 3));
        }
        return result;
    }
};


//pid
class SteeringController {
public:
    // 构造函数：设置车辆参数和PID参数
    SteeringController(double wheelbase, double lookahead_distance,
        double kp, double ki, double kd)
        : wheelbase_(wheelbase),
        lookahead_distance_(lookahead_distance),
        kp_(kp), ki_(ki), kd_(kd),
        integral_error_(0.0), prev_error_(0.0)
    {}

    // 归一化角度到 [-pi, pi]
    double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 纯跟踪算法：根据车辆当前位置和目标点计算期望转角
    // 参数：车辆坐标 (x, y)，航向角 theta（单位：弧度），目标点 (targetX, targetY)
    // 返回：期望的前轮转角（单位：弧度）
    double purePursuitSteering(double x, double y, double theta, double targetX, double targetY) {
        // 计算车辆与目标点连线的方向角
        double theta_target = std::atan2(targetY - y, targetX - x);
        // 计算车辆与目标方向之间的偏差角（归一化到[-pi, pi]）
        double alpha = normalizeAngle(theta_target - theta);
        // 根据纯跟踪公式计算曲率：κ = 2*sin(α)/L_d
        double kappa = 2.0 * std::sin(alpha) / lookahead_distance_;
        // 根据车辆几何模型，期望转角 δ_des = arctan(L * κ)
        double delta_des = std::atan(wheelbase_ * kappa);
        return delta_des;
    }

    // 结合纯跟踪和PID控制计算最终前轮转角
    // 输入参数：
    //   x, y                - 车辆坐标
    //   theta               - 车辆航向角（弧度）
    //   v                   - 车辆速度（此处未直接使用，便于后续扩展）
    //   targetX, targetY    - 目标点坐标
    //   currentSteeringAngle- 当前实际前轮转角（弧度）
    //   dt                  - 控制周期时间（秒）
    // 返回：最终前轮转角控制命令（弧度）
    double computeSteeringCommand(double x, double y, double theta, double v,
        double targetX, double targetY, double currentSteeringAngle, double dt) {
        // 1. 根据纯跟踪算法计算期望转角
        double delta_des = purePursuitSteering(x, y, theta, targetX, targetY);

        // 2. 计算转角误差：期望转角与当前实际转角之间的差值
        double error = delta_des - currentSteeringAngle;

        // 3. 利用PID控制器修正误差
        integral_error_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double pid_output = kp_ * error + ki_ * integral_error_ + kd_ * derivative;
        prev_error_ = error;

        // 4. 组合PID输出得到最终前轮转角控制命令
        // 这里可以理解为在当前转角上加上PID修正量
        double finalSteeringCommand = currentSteeringAngle + pid_output;
        return finalSteeringCommand;
    }

public:
    double wheelbase_;             // 车辆轴距
    double lookahead_distance_;    // 前视距离

    // PID参数
    double kp_;
    double ki_;
    double kd_;

    // PID内部状态
    double integral_error_;
    double prev_error_;
};

double wheelbase = 0.855;          // 车辆轴距（单位：米）
double lookahead_distance = 2; // 前视距离（单位：米）
double kp = 1.0, ki = 0.1, kd = 0.5;  // PID控制器参数

// 创建控制器对象
SteeringController controller(wheelbase, lookahead_distance, kp, ki, kd);
double targetX = 3, targetY = 0;


// 基于路径的速度规划函数
// 参数：
//   targetX, targetY：目标点相对车辆的坐标（单位：米）
//   v_max：最大允许速度（单位：m/s）
//   lambda：曲率影响系数（无量纲），数值越大，曲率对速度影响越明显
//   lookahead_distance：前视距离 L_d（单位：米），用于计算曲率
//
// 返回值：推荐速度（单位：m/s）
double pathBasedSpeedPlanning(double targetX, double targetY,
    double v_max, double lambda, double lookahead_distance) {
    // 计算目标点与车辆 x 轴的夹角 alpha（单位：弧度）
    double alpha = std::atan2(targetY, targetX);

    // 根据纯追踪公式计算曲率 κ = 2*sin(alpha)/L_d
    double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

    // 根据曲率计算推荐速度：曲率越大，速度应越低
    double recommendedSpeed = v_max / (1 + lambda * std::fabs(kappa));

    // 确保速度不低于 0
    return std::max(0.0, recommendedSpeed);
}


// 参数设置
double v_max = 3;              // 最大速度，15 m/s
double lambda = 10;              // 调整系数，依据实际调试进行设置

// 定义全局常量
const double Kp = 1.0;
const double Ki = 0.1;
const double Kd = 0.01;
const double dtt = 0.1;  // 时间步长
const double control_min = -1.0; // 控制信号下限
const double control_max = 1.0;  // 控制信号上限

// 全局变量
double integral = 0;        // 积分项
double previous_error = 0;  // 上一个误差


double control(double setpoint, double measurement) {
    // 计算误差
    double error = setpoint - measurement;

    // 计算积分和微分
    integral += error * dtt;
    double derivative = (error - previous_error) / dtt;

    // PID 输出
    double output = Kp * error + Ki * integral + Kd * derivative;

    // 限制控制信号在上下限之间
    if (output > control_max) {
        output = control_max;
    }
    else if (output < control_min) {
        output = control_min;
    }

    // 更新前一个误差
    previous_error = error;

    return output;
}


//mpc部分
const int N = 7;   //预测点个数
//const double dt = 0.5; // 时间步长
const double Lf = 0.855; // 车辆前轴到质心的距离
//const double ref_v = 1.5; // 参考速度

// 计算角度（基于斜率）x
MX jisuanxielv_casadi(tiaoyang& tyx, const MX& x) {
    MX slope = tyx.xielv(x);
    MX angleInRadians = atan(slope);
    return angleInRadians;
}



vector<pair<double, double>> xMPC_Solve(Eigen::VectorXd x0, tiaoyang& tyx, double xdest, double ydest) {
    // 定义控制输入符号变量
    MX delta = MX::sym("delta", N - 1);
    //MX a = MX::sym("a", N - 1);

    vector<MX> X(N), Y(N), Psi(N), V(N), Cte(N), Epsi(N);

    // 初始化状态变量
    X[0] = x0[0];
    Y[0] = x0[1];
    Psi[0] = x0[2];
    V[0] = x0[3];
    Cte[0] = x0[4];
    Epsi[0] = x0[5];

    // 状态更新
    for (int t = 1; t < N; t++) {
        MX psi0 = jisuanxielv_casadi(tyx, X[t - 1]);


        X[t] = X[t - 1] + V[t - 1] * cos(Psi[t - 1]) * dt;
        //cout << "X" << t << ":" << X[t] << endl;
        Y[t] = Y[t - 1] + V[t - 1] * sin(Psi[t - 1]) * dt;
        Psi[t] = Psi[t - 1] + V[t - 1] * delta(t - 1) * dt / Lf;
        V[t] = V[t - 1];

        Cte[t] = tyx.zhicasadi(X[t - 1]) - Y[t - 1] + V[t - 1] * sin(Epsi[t - 1]) * dt;
        //cout << "cte" << t << ":" << Cte[t] << endl;
        Epsi[t] = Psi[t - 1] - psi0 + V[t - 1] * delta(t - 1) * dt / Lf;
    }

    // 目标函数
    MX obj = 0;
    for (int t = 0; t < N - 1; t++) {
        obj += 30 * pow(Cte[t], 2) + 1 * pow(Epsi[t], 2);
        obj += 2 * pow(delta(t), 2);
        obj += pow(X[t] - xdest, 2) + pow(Y[t] - ydest, 2);


    }

    for (int t = 0; t < N - 2; t++) {
        obj += 5 * pow(delta(t + 1) - delta(t), 2);

    }
    obj += pow(delta(0) - x0[3], 2);

    // 约束条件
    vector<MX> constraints;
    for (int t = 1; t < N; t++) {
        constraints.push_back(X[t] - (X[t - 1] + V[t - 1] * cos(Psi[t - 1]) * dt));
        constraints.push_back(Y[t] - (Y[t - 1] + V[t - 1] * sin(Psi[t - 1]) * dt));
        constraints.push_back(Psi[t] - (Psi[t - 1] + V[t - 1] * delta(t - 1) * dt / Lf));
        constraints.push_back(V[t] - V[t - 1]);
    }

    // 变量、目标函数和约束

    MX g = vertcat(constraints);
    MXDict nlp = { {"x", delta}, {"f", obj}, {"g", g} };

    // 求解器设置da
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.max_iter"] = 1000;
    opts["ipopt.tol"] = 1e-6;
    opts["ipopt.print_level"] = 0; // 禁用 Ipopt 的打印日志
    opts["ipopt.sb"] = "yes"; // 禁用屏幕输出（silent mode）
    //opts["print_time"] = false; // 禁止 CasADi 打印时间信息
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // 初始值、边界和约束
    int n_vars = delta.size1();
    int n_constraints = g.size1();

    double delta_lower_bound = -0.2 * M_PI;
    double delta_upper_bound = 0.2 * M_PI;

    vector<double> vars_lowerbound(n_vars, -1e20);
    vector<double> vars_upperbound(n_vars, 1e20);

    for (int i = 0; i < N - 1; i++) {
        vars_lowerbound[i] = delta_lower_bound;
        vars_upperbound[i] = delta_upper_bound;
    }



    vector<double> constraints_lowerbound(n_constraints, -0.1);
    vector<double> constraints_upperbound(n_constraints, 0.1);


    // 求解
    DMDict arg = {
        {"x0", vector<double>(n_vars, 0.1)},
        {"lbx", vars_lowerbound},
        {"ubx", vars_upperbound},
        {"lbg", constraints_lowerbound},
        {"ubg", constraints_upperbound},
    };

    DMDict sol = solver(arg);

    double cost(sol.at("f"));
    //cout << "cost" << cost << endl;
    // 提取求解结果：控制输入 delta 和 a

    DM sol_values = sol["x"];
    vector<double> sol_vec = sol_values.nonzeros();

    // 保存未来9个点的轨迹坐标
    vector<pair<double, double>> predicted_trajectory;

    double xt_value = x0[0];
    double yt_value = x0[1];
    double psi_value = x0[2];
    double v_value = x0[3];

    for (int t = 0; t < N - 1; t++) {
        double delta_t = sol_vec[t];


        xt_value += v_value * cos(psi_value) * dt;
        yt_value += v_value * sin(psi_value) * dt;
        psi_value += v_value * delta_t * dt / Lf;
        v_value += 0;

        predicted_trajectory.push_back({ xt_value, yt_value });
    }
    predicted_trajectory.push_back({ sol_vec[0] ,0 });  //输出的当前delta
    return predicted_trajectory;
}



class MyCPlusPlusNode {
public:
    ros::NodeHandle nh_;
    ros::Subscriber zhuitong_subscription_;
    ros::Publisher publisher_kongzhi_;
    ros::Timer timer_;
    
    std::vector<std::pair<double, double>> l_xy;
    std::vector<std::pair<double, double>> r_xy;
    double kongzhi_v;
    double kongzhi_steer;

    MyCPlusPlusNode() {

        kongzhi_steer=0;
        kongzhi_v=0;
        // 订阅视觉节点发布的左侧数据
        zhuitong_subscription_ = nh_.subscribe("zhuitong_xy_topic", 10, &MyCPlusPlusNode::zhuitongCallback, this);
        
        // 控制信号发布节点
        publisher_kongzhi_ = nh_.advertise<std_msgs::Float64MultiArray>("kongzhi_topic", 10);
        
        // 定时器：每秒发布一次数据
        timer_ = nh_.createTimer(ros::Duration(0.1), &MyCPlusPlusNode::publisher_data_, this);
        
        //ROS_INFO("Trajectory node initialized, waiting for visual data...");
    }

    void publisher_data_(const ros::TimerEvent&) {
        std_msgs::Float64MultiArray msg_data;
        msg_data.data.push_back(kongzhi_v);
        msg_data.data.push_back(kongzhi_steer);
        publisher_kongzhi_.publish(msg_data);
        //ROS_INFO("Published v and steer data");
    }

    void zhuitongCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        l_xy.clear();
        r_xy.clear();
        
        // 检查数据总数是否为3的倍数
        if (msg->data.size() % 3 != 0) {
            ROS_ERROR("接收到的数据个数不是3的倍数!");
            return;
        }

        // 每3个数据为一组：第一个为标志位，后两个为坐标值
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            int flag = static_cast<int>(msg->data[i]);
            double y = (msg->data[i + 1]/100-6.6)/10;  //畸变值0.7393
            double x = (msg->data[i + 2]*10)/10;

            y=-y*x/3.2*5;
            
            //cout<<"x"<<x<<endl;
            //cout<<"y"<<y<<endl;
            

            if (flag == 1) {
                l_xy.push_back(std::make_pair(x, y));
            } else if (flag == 2) {
                r_xy.push_back(std::make_pair(x, y));
            } else {
                ROS_WARN("收到未知标志位: %d", flag);
            }
        }

        //ROS_INFO("解析完成: 左边点数 = %zu, 右边点数 = %zu", l_xy.size(), r_xy.size());
       processData();
    }

    void processData() {
        // auto start = high_resolution_clock::now();
        //     // 这里调用你的处理逻辑，例如构造 zhuitong 对象
            zhuitong z;
            z.l_xy = l_xy;
            z.r_xy = r_xy;
            // 此处可以继续调用处理函数，比如计算轨迹等
            // 初始化蓝色和红色锁桶数据
        // 初始化蓝色和红色锥桶数据---急弯红色过近处
        // z.r_xy = {
        //     {-4.46, -21.11}, {-1.40, -21.34}, {1.17, -20.36}, {2.63, -18.45}, {3.05, -16.21},
        //     {2.53, -14.07}, {1.19, -12.06}, {-0.47, -9.88}, {-4.17, -4.91}, {-4.92, -2.35},
        //     {-4.24, 0.27}, {-2.29, 2.03}, {0.13, 2.57}, {2.54, 1.80}, {4.54, 0.08},
        //     {11.68, -6.92}, {15.25, -10.41}, {8.11, -3.42}, {-2.40, -7.46}
        // };

        // z.l_xy = {
        //     {-1.47, -24.83}, {2.40, -23.73}, {5.38, -20.89}, {6.49, -17.94}, {5.76, -12.91},
        //     {4.21, -10.38}, {2.49, -8.16}, {0.70, -5.71}, {-1.16, -3.16}, {-1.04, -1.32},
        //     {0.53, -1.05}, {2.14, -2.36}, {5.71, -5.86}, {9.28, -9.36}, {12.86, -12.86},
        //     {6.52, -15.68}
        // };

        // 初始化蓝色和红色锥桶数据2---过弯直线
        // z.r_xy = {
        //     {2.37, 20.76}, {0.32, 18.06}, {-1.43, 15.83}, {-3.36, 13.39}, {-5.12, 11.01},
        //     {-6.83, 8.48}, {-7.25, 5.76}, {-6.31, 3.54}, {-4.58, 2.07}, {-2.46, 1.46},
        //     {-0.05, 1.62}, {2.66, 1.99}, {8.81, 2.74}, {11.41, 2.12}, {13.35, 0.24},
        //     {13.91, -2.33}, {13.19, -4.70}, {11.32, -6.41}, {8.84, -7.29}, {-0.78, -10.03},
        //     {-15.20, -14.14}, {-10.39, -12.77}, {-5.58, -11.40}, {4.03, -8.66}, {5.73, 2.47}
        // };

        // z.l_xy = {
        //     {-4.29, 17.89}, {-6.10, 15.49}, {-8.02, 12.96}, {-9.83, 10.26}, {-10.79, 6.36},
        //     {-9.79, 2.36}, {-7.78, -0.07}, {-3.05, -1.93}, {-0.09, -1.83}, {2.69, -1.43},
        //     {5.71, -1.09}, {8.85, -0.74}, {10.39, -1.76}, {9.84, -3.26}, {7.91, -4.01},
        //     {3.10, -5.37}, {-1.71, -6.74}, {-6.52, -8.11}, {-11.33, -9.48}, {-16.14, -10.85},
        //     {-5.84, -1.21}
        // };

        // 初始化蓝色和红色锥桶数据3---少点过弯
        // z.r_xy = {
        //     {-15.15, 12.41}, {-16.47, 9.59}, {-17.64, 6.88}, {-18.73, 4.02}, {-18.52, 1.28},
        //     {-17.10, -0.67}, {-15.07, -1.70}, {-12.87, -1.82}, {-10.56, -1.11}, {-8.01, -0.13},
        //     {-2.19, 2.01}, {0.48, 2.00}, {2.80, 0.61}, {3.93, -1.76}, {3.76, -4.23},
        //     {2.34, -6.33}, {0.13, -7.76}, {-8.61, -12.62}, {-12.98, -15.05}, {-4.24, -10.19},
        //     {-5.13, 1.04}
        // };

        // z.l_xy = {
        //     {-19.62, 11.01}, {-20.91, 8.12}, {-22.06, 5.08}, {-22.10, 1.06}, {-20.22, -2.61},
        //     {-17.71, -4.52}, {-12.68, -5.25}, {-9.81, -4.47}, {-7.20, -3.45}, {-4.33, -2.43},
        //     {-1.36, -1.38}, {0.37, -2.02}, {0.18, -3.59}, {-1.54, -4.77}, {-5.90, -7.20},
        //     {-10.27, -9.63}, {-14.64, -12.06}, {-15.55, -5.18}
        // };

        //有弯不缺点数据

    //    z.r_xy = {
    //     {-12.57, -1.10}, {-7.74, 0.22}, {-2.92, 1.55}, {-0.01, 2.07}, {3.58, 0.87}, {5.64, -1.54}, {6.47, -4.96}, {5.63, -7.75},
    //         {2.77, -10.90}, {0.56, -13.00}, {-4.36, -17.65}, {-6.92, -20.12}, {-17.39, -2.43}, {-1.92, -15.34}, {-8.18, -21.73}
    //     };

    //     z.l_xy = {
    //         {-16.48, -5.73}, {-6.84, -3.08}, {-2.02, -1.75}, {0.09, -1.44}, {1.60, -1.99}, {2.67, -3.30}, {2.40, -6.18},
    //         {0.40, -8.31}, {-1.76, -10.39}, {-4.25, -12.76}, {-6.68, -15.05}, {-9.15, -17.42}, {-12.42, -23.17}, {-11.39, -20.44},
    //         {-11.66, -4.40}
    //     };


        //有干扰点
        // z.r_xy = {

        //     {0.36,-2.62},{4.91,-0.38},{6.84,1.88},{8.42,4.19},{11.61,1.84},{9.38,-2.64},{7.14,-7.11},{4.91,-11.59},{2.51,-1.95}
        // };

        // z.l_xy = {
        //     {0.94,1.19},{2.82,2.34},{4.32,4.23},{5.88,6.48}
        // };

    //      z.r_xy = {
    //    {1.29, -1.72}, {11.29, -1.72}, {16.29, -1.72}, {18.40, -1.98}, {19.71, -2.91},
    //    {20.40, -4.46}, {19.38, -7.16}, {16.88, -8.69}, {14.25, -10.12}, {11.22, -11.74},
    //    {8.27, -13.31}, {5.26, -14.94}, {0.58, -19.62}, {2.30, -17.26}, {6.29, -1.72}
    //      };

    //      z.l_xy = {
    //          {6.29, 1.70}, {11.29, 1.70}, {16.29, 1.70}, {19.24, 1.43}, {18.48, -11.81},
    //          {15.79, -13.25}, {9.82, -16.43}, {6.69, -18.13}, {1.29, 1.70}, {12.78, -14.85},
    //          {5.05, -19.35}
    //      };


         //直线
         /*z.l_xy = {
         {2.03, 1.68},
         {2.03, 1.68},
         {7.03, 1.63},
         {7.03, 1.63},
         {12.03, 1.59},
         {12.03, 1.59},
         {17.03, 1.55},
         {17.03, 1.55},
         {20.61, -19.53},
         {20.61, -19.53}
         };

         z.r_xy = {
             {2.00, -1.74},
             {7.00, -1.79},
             {12.00, -1.83},
             {17.00, -1.87},
             {16.14, -19.76},
             {17.88, -17.42}
         };*/


             z.xy = z.l_xy;
 z.xy.insert(z.xy.end(), z.r_xy.begin(), z.r_xy.end());

//cout<<"z.xy"<<z.xy.size()<<endl;
//  // 执行每一步并输出
 //z.shaixuan();

//  //z.youxiaozhuitong();
//  //z.youxiaozhuitong_tsp();

z.youxiaozhuitong_weizhi();


// // //  /*auto stop = high_resolution_clock::now();
// // //  auto duration = duration_cast<milliseconds>(stop - start);*/
  z.lujingdian();



 tong_blue = z.r_xy;
 tong_red = z.l_xy;

 tong_r = z.l_xy_good;
 tong_b = z.r_xy_good;

// // //  /*tong_r = z.l_xy;
// // //  tong_b = z.r_xy;*/

 lujing = z.lujing_dian;
 lujing2 = z.lujing_dian2;

 tiaoyang tyx(lujing2);


tyquanxian.clear();

 for (int i = 0; i < 100; i++) {
     double t = static_cast<double>(i) / 100; // 计算 t，从 0 到 1
     double x = (1 - t) * tyx.dian[0].first + t * tyx.dian[1].first; // 插值计算 x
     //double y = tyx.a[0] + tyx.b[0] * (x - tyx.dian[0].first) +
     //    tyx.c[0] * pow(x - tyx.dian[0].first, 2) +
     //    tyx.d[0] * pow(x - tyx.dian[0].first, 3); // 计算 y
     double y = tyx.zhi(x);
     tyquanxian.push_back(std::pair<double, double>{x, y}); // 保存点
 }

 for (int i = 0; i < 100; i++) {
     double t = static_cast<double>(i) / 100; // 计算 t，从 0 到 1
     double x = (1 - t) * tyx.dian[1].first + t * tyx.dian[2].first; // 插值计算 x
     double y = tyx.a[1] + tyx.b[1] * (x - tyx.dian[1].first) +
         tyx.c[1] * pow(x - tyx.dian[1].first, 2) +
         tyx.d[1] * pow(x - tyx.dian[1].first, 3); // 计算 y
     tyquanxian.push_back(std::pair<double, double>{x, y}); // 保存点
 }

 for (int i = 0; i < 100; i++) {
     double t = static_cast<double>(i) / 100; // 计算 t，从 0 到 1
     double x = (1 - t) * tyx.dian[2].first + t * tyx.dian[3].first; // 插值计算 x
     double y = tyx.a[2] + tyx.b[2] * (x - tyx.dian[2].first) +
         tyx.c[2] * pow(x - tyx.dian[2].first, 2) +
         tyx.d[2] * pow(x - tyx.dian[2].first, 3); // 计算 y
     tyquanxian.push_back(std::pair<double, double>{x, y}); // 保存点
 }

 for (int i = 0; i < 100; i++) {
     double t = static_cast<double>(i) / 100; // 计算 t，从 0 到 1
     double x = (1 - t) * tyx.dian[3].first + t * tyx.dian[4].first; // 插值计算 x
     double y = tyx.a[3] + tyx.b[3] * (x - tyx.dian[3].first) +
         tyx.c[3] * pow(x - tyx.dian[3].first, 2) +
         tyx.d[3] * pow(x - tyx.dian[3].first, 3); // 计算 y
     tyquanxian.push_back(std::pair<double, double>{x, y}); // 保存点
 }


// // // //  auto stop = high_resolution_clock::now();
// // // //  auto duration = duration_cast<milliseconds>(stop - start);

// // //  //cout << "程序执行时间: " << duration.count() << " 毫秒" << endl;


// // //  //规划模块

//  //目标点
  for (int i = 0; i < lujing.size(); i++)
 {
     if (distance(lujing2[i], { 0,0 }) > lookahead_distance)
     {
         if (distance({ targetX,targetY }, lujing2[i]) > 0.3)
         {
             controller.integral_error_ = 0;
             controller.prev_error_ = 0;
         }
         targetX = lujing2[i].first;
         targetY = lujing2[i].second;
         break;
     }
 }

  // 示例车辆状态
  double x = 0.0, y = 0.0;              // 车辆坐标
  double theta = 0.0;                   // 车辆航向角（单位：弧度）
  double v = 2;                      // 车辆速度（单位：m/s）
  double currentSteeringAngle = 0.1;    // 当前前轮转角（单位：弧度）



//   // 控制周期
  double dt = 0.1;

 //速度规划
  double mubiaov = pathBasedSpeedPlanning(targetX, targetY, v_max, lambda, lookahead_distance);;
  if (mubiaov <= 1.5) mubiaov = 1.5;
  //cout << "mubiaosudu" << mubiaov << endl;
  

  double control_signal = control(mubiaov, v);
  //cout << "guihuavc" << control_signal << endl;


//   //pid横向
  

//   // 计算最终前轮转角控制命令
// //   double finalSteering = controller.computeSteeringCommand(x, y, theta, v,
// //       targetX, targetY, currentSteeringAngle, dt);

  

  


// //   //mpc横向
  MX xdest = lujing2[4].first;
  MX ydest = lujing2[4].second;
  dt = (double)(sqrt(xdest * xdest + ydest * ydest) / v / N) / 0.9;  //自己设计的革新

  vector<pair<double, double>> predicted_trajectory;

  Eigen::VectorXd x0(6); x0 << 0.0, 0.0, 0.0, v, 0, 0;   //车辆初始参数

  predicted_trajectory = xMPC_Solve(x0, tyx, (double)xdest, (double)ydest);


  double finalSteering = predicted_trajectory[N - 1].first;
  

  predicted_trajectory.pop_back();


  weilai_lujing = predicted_trajectory;

  

std::cout << "最终前轮转角控制命令: " << -finalSteering << " (弧度)" << std::endl;
kongzhi_steer=finalSteering;



cout<<"z.xy.size()::"<<z.xy.size()<<endl;
if(z.xy.size()>3)
{
    kongzhi_v=1;
}
else{
    kongzhi_v=0;
}

            

       

    
    }
};

int main(int argc, char* argv[]) {
    // 创建一个线程来运行 xianchenghanshu()
    std::thread drawThread(xianchenghanshu);
    drawThread.detach();
    
    ros::init(argc, argv, "my_cplusplus_node");
    MyCPlusPlusNode node;
    ros::spin();
    return 0;
}





