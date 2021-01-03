//
// Created by 安頔 on 2020/12/14.
//

#ifndef SEAM_CARVING_SEAMCARVE_H
#define SEAM_CARVING_SEAMCARVE_H

#include <opencv2/opencv.hpp>
#include <ctime>

enum ENERGY{
    kGrad,
    kEntropy,
    kHOG
};

class SeamCarve {
public:
    cv::Mat img_in;
    cv::Mat remove_mask;
    cv::Mat img_out;

    explicit SeamCarve(int max_h, int max_w);
    void set_img_in(const char* img_path);
    void set_forward(bool is_forward);
    void set_remove_mask(const char* img_path);
    void set_optimal_order(bool optimal);
    void show_img_out() const;
    void store_img_out(const char* img_path) const;
    void run(int iter_h, int iter_w);
    void remove_object();
    ~SeamCarve();

    void set_energy_choice(enum ENERGY choice);

private:
    static int getId(int i, int j, int w);
    static double delEnergy(const cv::Mat& gray, int x, int y, int u, int v);
    void getDpResult(const cv::Mat& gray);
    double removeHorizon(int iter);
    void insertHorizon(int iter);
    void computeEnergy(const cv::Mat& gray);

    enum ENERGY energy_choice{kGrad};
    int max_h, max_w;
    double *dp;
    double *energy_map;
    bool *protect;
    bool *remove_pos;
    std::pair<int, int> *dp_source;
    const double inf = 1e10;
    bool forward;
    bool remove;
    bool is_optimal;

};


#endif //SEAM_CARVING_SEAMCARVE_H
