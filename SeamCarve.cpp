//
// Created by 安頔 on 2020/12/14.
//

#include "SeamCarve.h"

SeamCarve::SeamCarve(int max_h, int max_w) {
    this->max_h = max_h;
    this->max_w = max_w;
    dp = new double[max_h*max_w];
    dp_source = new std::pair<int, int>[max_h*max_w];
    energy_map = new double[max_h*max_w];
    protect = new bool[max_h*max_w]{false};
    remove_pos = new bool[max_h*max_w]{false};
    forward = false;
}

void SeamCarve::set_energy_choice(enum ENERGY choice) {
    energy_choice = choice;
}

SeamCarve::~SeamCarve(){
    delete [] dp;
    delete [] dp_source;
    delete [] energy_map;
    delete [] protect;
    delete [] remove_pos;
}

void SeamCarve::set_forward(bool is_forward) {
    this->forward = is_forward;
}

void SeamCarve::set_img_in(const char *img_path) {
    img_in = cv::imread(img_path);
}

void SeamCarve::show_img_out() const {
    cv::imshow("out", img_out);
    cv::waitKey(0);
}

void SeamCarve::run(int iter_h, int iter_w) {
    img_out = img_in.clone();
    int goal_h = iter_h+img_in.rows;
    int goal_w = iter_w+img_in.cols;
    if(goal_h <= 0 || goal_h > max_h || goal_w <= 0 || goal_w > max_w){
        printf("ERROR: THE SCALE IS NOT RIGHT\n");
        exit(0);
    }
    if(iter_w != 0){
        if(iter_w < 0)removeHorizon(iter_w);
        else insertHorizon(iter_w);
    }
    if(iter_h != 0){
        img_out = img_out.t();
        if(iter_h < 0)removeHorizon(iter_h);
        else insertHorizon(iter_h);
        img_out = img_out.t();
    }
}

void SeamCarve::remove_object() {
    img_out = img_in.clone();
    int origin_h = img_in.rows;
    int origin_w = img_in.cols;
    remove = false;
    for(int i = 0; i < remove_mask.cols; ++i){
        for(int j = 0; j < remove_mask.rows; ++j){
            if(remove_mask.at<cv::Vec3b>(j, i) != cv::Vec3b(0, 0, 0)){
                remove_pos[getId(i, j, img_out.rows)] = true;
                remove = true;
            }else remove_pos[getId(i, j, img_out.rows)] = false;
        }
    }
    while(remove){
        removeHorizon(-1);
        remove = false;
        for(int i = 0; i < remove_mask.cols; ++i){
            for(int j = 0; j < remove_mask.rows; ++j){
                if(remove_mask.at<cv::Vec3b>(j, i) != cv::Vec3b(0, 0, 0)){
                    remove_pos[getId(i, j, img_out.rows)] = true;
                    remove = true;
                }else remove_pos[getId(i, j, img_out.rows)] = false;
            }
        }
    }
    while(img_out.cols != origin_w){
        int goal_w = std::min(origin_w, (int)(img_out.cols*1.5));
        insertHorizon(goal_w-img_out.cols);
    }
}

void SeamCarve::insertHorizon(int iter) {
    clock_t startTime, endTime;
    startTime = clock();
    int real_iter = iter;
    int step = (iter >> 1);
    while(real_iter){
        int iter_step = std::min(real_iter, step);
        real_iter -= iter_step;
        for(int i = 0; i < img_out.cols; ++i){
            for(int j = 0; j < img_out.rows; ++j){
                protect[getId(i, j, img_out.rows)] = false;
            }
        }
        cv::Mat gray_out;
        cv::cvtColor(img_out, gray_out, cv::COLOR_BGR2GRAY);
        int goal_w = img_out.cols+iter_step;
        while(iter_step--){
            printf("iter = %d\n", iter_step);
            endTime = clock();
            std::cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
            computeEnergy(gray_out);
            getDpResult(gray_out);
            int ans_x, ans_y;
            ans_x = 0; ans_y = gray_out.rows-1;
            double ans = dp[getId(ans_x, ans_y, gray_out.rows)];
            for(int i = 1; i < gray_out.cols; ++i){
                double tmp = dp[getId(i, gray_out.rows-1, gray_out.rows)];
                if(tmp < ans){
                    ans = tmp;
                    ans_x = i;
                }
            }
            while(ans_x != -1){
                protect[getId(ans_x, ans_y, gray_out.rows)] = true;
                //printf("%d %d\n", ans_x, ans_y);
                ans_x = dp_source[getId(ans_x, ans_y, gray_out.rows)].first;
                ans_y--;
            }
        }
        cv::Mat img_tmp = cv::Mat(img_out.rows, goal_w, img_out.type());
        for(int i = 0; i < img_out.rows; ++i) {
            int offset = 0;
            for(int j = 0; j < img_out.cols; ++j){
                if(protect[getId(j, i, gray_out.rows)]) {
                    if(j > 0){
                        cv::Vec3s left_color = img_out.at<cv::Vec3b>(i, j-1);
                        cv::Vec3s right_color = img_out.at<cv::Vec3b>(i, j);
                        img_tmp.at<cv::Vec3b>(i, j+offset) = (left_color+right_color)/2;
                    }else img_tmp.at<cv::Vec3b>(i, j+offset) = img_out.at<cv::Vec3b>(i, j);
                    offset++;
                }img_tmp.at<cv::Vec3b>(i, j+offset) = img_out.at<cv::Vec3b>(i, j);
            }
        }
        img_out = img_tmp.clone();
    }
}

void SeamCarve::removeHorizon(int iter) {
    cv::Mat gray_out;
    cv::cvtColor(img_out, gray_out, cv::COLOR_BGR2GRAY);
    int real_iter = -iter;
    clock_t startTime, endTime;
    startTime = clock();
    while(real_iter--){
        printf("iter = %d\n", real_iter);
        endTime = clock();
        std::cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
        computeEnergy(gray_out);
        getDpResult(gray_out);
        int ans_x, ans_y;
        ans_x = 0; ans_y = gray_out.rows-1;
        double ans = dp[getId(ans_x, ans_y, gray_out.rows)];
        for(int i = 1; i < gray_out.cols; ++i){
            double tmp = dp[getId(i, gray_out.rows-1, gray_out.rows)];
            if(tmp < ans){
                ans = tmp;
                ans_x = i;
            }
        }
        while(ans_x != -1){
            for(int i = ans_x; i < gray_out.cols-1; ++i){
                gray_out.at<uchar>(ans_y, i) = gray_out.at<uchar>(ans_y, i+1);
                img_out.at<cv::Vec3b>(ans_y, i) = img_out.at<cv::Vec3b>(ans_y, i+1);
                if(remove)remove_mask.at<cv::Vec3b>(ans_y, i) = remove_mask.at<cv::Vec3b>(ans_y, i+1);
            }
            ans_x = dp_source[getId(ans_x, ans_y, gray_out.rows)].first;
            ans_y--;
        }
        gray_out = gray_out(cv::Rect(0, 0, gray_out.cols-1, gray_out.rows));
    }
    img_out = img_out(cv::Rect(0, 0, img_out.cols+iter, img_out.rows));
    if(remove)remove_mask = remove_mask(cv::Rect(0, 0, img_out.cols+iter, img_out.rows));
}

void SeamCarve::computeEnergy(const cv::Mat& gray) {
    cv::Mat grad_x, grad_y;
    cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
    for(int i = 0; i < gray.cols; ++i){
        for(int j = 0; j < gray.rows; ++j){
            energy_map[getId(i, j, gray.rows)] =
                    std::fabs(grad_x.at<float>(j, i))+std::fabs(grad_y.at<float>(j, i));
        }
    }
    switch(energy_choice){
        case kGrad:
            break;
        case kEntropy: {
            int window_len = 9;
            int window_size = window_len * window_len;
            int window_len_half = (window_len >> 1);
            auto *compute_entropy = new double[window_size + 1];
            for (int i = 1; i <= window_size; ++i) {
                double p = (double) (i) / window_size;
                compute_entropy[i] = -p * std::log2(p);
            }
            int vis_cnt[256];
            std::set<int> color;
            for (int i = 0; i < gray.cols; ++i) {
                int window_cnt = 0;
                int u_st = std::max(0, i - window_len_half);
                int u_ed = std::min(i + window_len_half + 1, gray.cols);
                int v_st = 0;
                int v_ed = std::min(window_len_half + 1, gray.rows);
                color.clear();
                for (int u = u_st; u < u_ed; ++u) {
                    for (int v = v_st; v < v_ed; ++v) {
                        int now_color = gray.at<uchar>(v, u);
                        if (color.find(now_color) == color.end()) {
                            vis_cnt[now_color] = 1;
                            color.insert(now_color);
                        } else vis_cnt[now_color]++;
                        window_cnt++;
                    }
                }
                int id = getId(i, 0, gray.rows);
                for (auto tmp: color) {
                    if (window_cnt == window_size)energy_map[id] += compute_entropy[vis_cnt[tmp]];
                    else {
                        energy_map[id] -= vis_cnt[tmp] * std::log2(vis_cnt[tmp]);
                    }
                }
                for (int j = 1; j < gray.rows; ++j) {
                    if (j > window_len_half) {
                        int del_row = j - window_len_half - 1;
                        window_cnt -= (u_ed - u_st);
                        for (int u = u_st; u < u_ed; ++u) {
                            int now_color = gray.at<uchar>(del_row, u);
                            vis_cnt[now_color]--;
                            if (vis_cnt[now_color] == 0)color.erase(now_color);
                        }
                    }
                    if (j < gray.rows - window_len_half) {
                        int add_row = j + window_len_half;
                        window_cnt += (u_ed - u_st);
                        for (int u = u_st; u < u_ed; ++u) {
                            int now_color = gray.at<uchar>(add_row, u);
                            if (color.find(now_color) == color.end()) {
                                color.insert(now_color);
                                vis_cnt[now_color] = 1;
                            } else vis_cnt[now_color]++;
                        }
                    }
                    id = getId(i, j, gray.rows);
                    for (auto tmp: color) {
                        //printf("%d %d\n", tmp, vis_cnt[tmp]);
                        if (window_cnt == window_size)energy_map[id] += compute_entropy[vis_cnt[tmp]];
                        else {
                            energy_map[id] -= vis_cnt[tmp] * std::log2(vis_cnt[tmp]);
                        }
                    }
                }
            }
            delete[] compute_entropy;
            break;
        }
        case kHOG: {
            int window_len = 11;
            int window_len_half = (window_len >> 1);
            cv::Mat grad_angle;
            cv::Mat grad;
            cv::cartToPolar(grad_x, grad_y, grad, grad_angle, true);
            int count_angle[9];
            for (int i = 0; i < gray.cols; ++i) {
                int window_cnt = 0;
                int u_st = std::max(0, i - window_len_half);
                int u_ed = std::min(i + window_len_half + 1, gray.cols);
                int v_st = 0;
                int v_ed = std::min(window_len_half + 1, gray.rows);
                for(int j = 0; j < 8; ++j)count_angle[j] = 0;
                for (int u = u_st; u < u_ed; ++u) {
                    for (int v = v_st; v < v_ed; ++v) {
                        int type = (int)(grad_angle.at<float>(v, u)/45.0);
                        count_angle[type]++;
                        window_cnt++;
                    }
                }
                int id = getId(i, 0, gray.rows);
                int mx = 0;
                for(int j = 0; j < 8; ++j)if(count_angle[j] > mx)mx = count_angle[j];
                assert(mx != 0);
                energy_map[id] = energy_map[id]*window_cnt/mx;
                for (int j = 1; j < gray.rows; ++j) {
                    if (j > window_len_half) {
                        int del_row = j - window_len_half - 1;
                        window_cnt -= (u_ed - u_st);
                        for (int u = u_st; u < u_ed; ++u) {
                            int type = (int)(grad_angle.at<float>(del_row, u)/45.0);
                            count_angle[type]--;
                        }
                    }
                    if (j < gray.rows - window_len_half) {
                        int add_row = j + window_len_half;
                        window_cnt += (u_ed - u_st);
                        for (int u = u_st; u < u_ed; ++u) {
                            int type = (int)(grad_angle.at<float>(add_row, u)/45.0);
                            count_angle[type]++;
                        }
                    }
                    id = getId(i, j, gray.rows);
                    mx = 0;
                    for(int t = 0; t < 8; ++t){
                        if(count_angle[t] > mx)mx = count_angle[t];
                    }
                    assert(mx != 0);
                    energy_map[id] = energy_map[id]*window_cnt/mx;
                }
            }
            break;
        }
    }
}

int SeamCarve::getId(int i, int j, int w) {
    return i*w+j;
}

void SeamCarve::store_img_out(const char *img_path) const {
    cv::imwrite(img_path, img_out);
}

void SeamCarve::getDpResult(const cv::Mat& gray) {
    for(int i = 0; i < gray.cols; ++i){
        int id = getId(i, 0, gray.rows);
        if(remove && remove_pos[id])dp[id] = -inf;
        else if(!protect[id])dp[id] = energy_map[id];
        else dp[id] = inf;
        dp_source[id] = std::make_pair(-1, -1);
    }
    for(int j = 1; j < gray.rows; ++j){
        for(int i = 0; i < gray.cols; ++i){
            int id = getId(i, j, gray.rows);
            if(remove && remove_pos[id])dp[id] = -inf;
            else if(!protect[id])dp[id] = energy_map[id];
            else dp[id] = inf;
            double tmp_min = dp[getId(i, j-1, gray.rows)];
            int src_x = i;
            int src_y = j-1;
            if(i > 0){
                double tmp = dp[getId(i-1, j-1, gray.rows)];
                if(forward){
                    tmp += delEnergy(gray, i, j-1, i-1, j);
                }
                if(tmp < tmp_min){
                    tmp_min = tmp;
                    src_x = i-1;
                }
            }
            if(i < gray.cols-1){
                double tmp = dp[getId(i+1, j-1, gray.rows)];
                if(forward){
                    tmp += delEnergy(gray, i, j-1, i+1, j);
                }
                if(tmp < tmp_min){
                    tmp_min = tmp;
                    src_x = i+1;
                }
            }
            dp[id] += tmp_min;
            if(forward){
                dp[id] += delEnergy(gray, std::max(0, i-1), j, std::min(gray.cols-1, i+1), j);
            }
            dp_source[id] = std::make_pair(src_x, src_y);
        }
    }
}

double SeamCarve::delEnergy(const cv::Mat &gray, int x, int y, int u, int v) {
    int gray_u, gray_v;
    //if(x < 0 || x >= gray.cols || y < 0 || y >= gray.rows)return 0;
    //if(u < 0 || u >= gray.cols || v < 0 || v >= gray.cols)return 0;
    gray_u = gray.at<uchar>(y, x);
    gray_v = gray.at<uchar>(v, u);
    return std::abs(gray_u-gray_v);
}

void SeamCarve::set_remove_mask(const char *img_path) {
    remove_mask = cv::imread(img_path);
}

