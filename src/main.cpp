//
// Created by waxz on 19-2-21.
//
#include <iostream>
#include <valarray>
#include <cpp_utils/ublas.h>
#include <cpp_utils/tbb_util.h>
#include <cpp_utils/ros_util.h>
#include <cpp_utils/eigen_util.h>

#include <cpp_utils/time.h>
#include <vector>
#include <boost/array.hpp>
#include <ros/ros.h>
#include <demo/grid.h>
#include <opencv2/opencv.hpp>
#include "test_speed.cpp"
#include <boost/random/geometric_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
namespace mp_util {

    struct mp_float {
        float v_valid;
        int v_log;

        void set(float v) {

            v_valid = v;
            v_log = 0;

        }

        mp_float() {
            set(0.0);
        }

        mp_float(float v) {
            set(v);
        }

        mp_float(const mp_float &v) {

            v_valid = v.v_valid;
            v_log = v.v_log;
        }

        mp_float &operator=(const float &v) {
            set(v);
            return *this;

        }

        void operator=(const mp_float &v) {

            v_valid = v.v_valid;
            v_log = v.v_log;

//        return *this;
        }

        mp_float operator*(float v) {
            mp_float res;
            res.v_log = v_log;
            res.v_valid = v_valid;

            res.v_valid *= v;
            if (fabs(res.v_valid) > 1e36 ||
                fabs(res.v_valid) < 1e-36) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(res.v_valid));

                res.v_valid /= std::pow(10, v_update);

                res.v_log += v_update;

            }

            return res;
        }

        mp_float operator*=(float v) {
            v_valid *= v;
            if (fabs(v_valid) > 1e35 ||
                fabs(v_valid) < 1e-35) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }
        }

        void prod_v(float v) {
            if (0 != v_valid * v) {
                v_valid *= v;
                if (fabs(v_valid) > 1e35 ||
                    fabs(v_valid) < 1e-35) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                    int v_update = int(std::log10(v_valid));

                    v_valid /= std::pow(10, v_update);

                    v_log += v_update;
                }
            } else {
                if (v_valid >= v) {
                    int v_update = int(std::log10(v));
                    v /= std::pow(10, v_update);

                    v_valid *= v;
                    v_log += v_update;

                }
            }
        }

        void prod_mpf(mp_float v) {
            {
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }
            {
                int v_update = int(std::log10(v.v_valid));

                v.v_valid /= std::pow(10, v_update);

                v.v_log += v_update;
            }

            v_valid *= v.v_valid;
            v_log += v.v_log;
            if (fabs(v_valid) > 1e35 ||
                fabs(v_valid) < 1e-35) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }

        }

        void add_mpf(mp_float v) {
            {
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;

                if (v_valid == 0.0) {

                    this->v_valid = v.v_valid;
                    this->v_log = v.v_log;
                    return;
                }
            }
            {
                int v_update = int(std::log10(v.v_valid));

                v.v_valid /= std::pow(10, v_update);

                v.v_log += v_update;
            }

            int sub = v.v_log - this->v_log;

//            v_valid *= v;
            if (sub > 0) {

                this->v_log = v.v_log;
                this->v_valid *= pow(0.1, sub);
                this->v_valid += v.v_valid;

            } else if (sub < 0) {

                this->v_valid += v.v_valid * pow(10, sub);

            } else if (0 == sub) {
                this->v_valid += v.v_valid;

            }

            if (fabs(v_valid) > 1e35 ||
                fabs(v_valid) < 1e-35) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }
        }

        void devide_mpf(mp_float v) {
            {
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }
            {
                int v_update = int(std::log10(v.v_valid));

                v.v_valid /= std::pow(10, v_update);

                v.v_log += v_update;
            }
            v_valid /= v.v_valid;
            v_log -= v.v_log;
            if (fabs(v_valid) > 1e35 ||
                fabs(v_valid) < 1e-35) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            }

        }
        mp_float operator*=(const mp_float &v) {
            v_valid *= v.v_valid;
            if (fabs(v_valid) > 1e36 ||
                fabs(v_valid) < 1e-36) {//|| (v_valid  < 1e-38 || v_valid  > -1e-38 ) || v_valid < -1e38 ){
                int v_update = int(std::log10(v_valid));

                v_valid /= std::pow(10, v_update);

                v_log += v_update;
            } else {
                v_log += v.v_log;

            }
        }


        bool operator>(const mp_float &rv) const {
            int sub = rv.v_log - this->v_log;

            if (rv.v_valid > 0 != this->v_valid > 0 || sub == 0) {
                return (rv.v_valid < this->v_valid);
            }
            if (rv.v_valid * this->v_valid != 0) {
                return (rv.v_valid * pow(10, sub) < this->v_valid);

            }


            return (rv.v_log < this->v_log) && (rv.v_valid < this->v_valid);
        }

        bool operator<(const mp_float &rv) const {
            return !(rv > *this);
        }

        void print() {
            std::cout << v_valid << " e " << v_log << std::endl;
        }

    };


    void pow_split(float a, float b, std::vector<float> &vc) {

        float p = pow(a, b);
        if (0 == p || std::isnan(p)) {
            pow_split(a, int(b * 0.5), vc);

            pow_split(a, b - int(b * 0.5), vc);

        } else {
            vc.push_back(b);
        }
    }

    mp_float mp_pow(float base, float exp) {
        mp_util::mp_float res = 1.0;

        std::vector<float> vc(0);
        pow_split(base, exp, vc);
        for (auto i : vc) {
            res.prod_v(pow(base, i));
        }
        return res;
    }

}


namespace std {
#pragma omp declare simd

    template<typename T>
    inline T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }

}

float normal_pdf(float x, float m, float s) {
//    static const float inv_sqrt_2pi = 0.3989422804014327;
//    float a = (x - m) / s;

//    inv_sqrt_2pi / s * std::exp(-0.5f * a * a);

    float pdf_gaussian = (1.0 / (s * sqrt(2 * M_PI))) * exp(-0.5 * pow((x - m) / s, 2.0));

    return pdf_gaussian;
}

struct GaussLUT {
    std::vector<float> data_vec;
    float m_max_x;
    size_t m_max_num;
    float *data;
    float scale;

    GaussLUT(float m, float s, float max_x, size_t max_num) : m_max_x(max_x), m_max_num(max_num) {
        data_vec = std::vector<float>(max_num, 0.0);
        float b = max_x / max_num;
        for (size_t i = 0; i < max_num; i++) {
            data_vec[i] = normal_pdf(i * b, m, s);
        }
        data = &(data_vec[0]);
        scale = m_max_num / m_max_x;

    }

    float operator()(float x) {
        size_t index = m_max_num * x / m_max_x;
        return data_vec[index];
    }


};


struct Partial {

    ublas_util::Transformation2d map_laser_pose;
    ublas::matrix<float> minv_beam;
    float weight;
//    mp::mpf_float match_weight;
    float match_weight;
    mp_util::mp_float mp_weight;
    mp_util::mp_float map_weight;


    std::valarray<float> Q;
    std::valarray<float> Q_;

    // index array, store cell index
    std::vector<int> occuPointIndex;
    std::vector<float> occuPointQ;
    std::vector<float> occuPointQ_;

    std::vector<int> freePointIndex;
    std::vector<float> freePointQ;
    std::vector<float> freePointQ_;
    // store update Q
    ublas::compressed_matrix<float> global_map;
    ublas::compressed_matrix<float> old_global_map;
    ublas::compressed_matrix<float> current_global_map;

    // store cell type
    ublas::compressed_matrix<char> global_map_cell;

    // local map
    ublas::compressed_matrix<char> local_map;

    std::valarray<int> erase_index1;
    std::valarray<int> erase_index2;

    std::valarray<int> P;

    // todo: cell transformation count [0,1,2,3,4,5]
    // free 0-0,0-1,1-1,1-0
    // occu 0-0,0-1,1-1,1-0
    // dyna 0-0,0-1,1-1,1-0
    std::valarray<int> m_chnage_cell_cnt;
    std::valarray<float> m_change_cell_prob;
    float m_normal_change_cell_prob;
    float m_normal_change_cell_prob_base;
    float m_map_update_weight;



    Partial() : minv_beam(ublas::matrix<float>(3, 3)), mp_weight(1.0),
                P(1000 * 20 * 2), Q(0.0f, 1000 * 20), Q_(0.0f, 1000 * 20), m_chnage_cell_cnt(0, 50),
                m_change_cell_prob(0.0, 12),
                global_map(300, 300, 300 * 300), old_global_map(300, 300, 300 * 300),
                current_global_map(300, 300, 300 * 300),
                erase_index1(2000), erase_index2(2000), m_map_update_weight(0.0), map_weight(1.0) {

    };

    void set_m_change_cell_prob(const std::valarray<float> &v) {
        m_normal_change_cell_prob_base = v.min();
        m_change_cell_prob = v / m_normal_change_cell_prob_base;
    }


    void reset_m_chnage_cell_cnt() {
        for (auto &i : m_chnage_cell_cnt) {
            i = 0;
        }
    }


};

class PartialManager {
private:
    size_t m_partial_num;
    std::vector<Partial> partials;

    // initial pose guass model

    // movement model gauss model

    // map resource path
    std::string global_map_path;
    std::string global_map_type_path;
    cv::Mat global_map;
    ublas::compressed_matrix<float> global_map_type;

public:
    PartialManager(size_t partial_num) : m_partial_num(partial_num), partials(1000) {

    }

    //get basic resource global map, cell type map
    void read_map() {

    }

    // create_initial partial cloud with initial pose
    // given robot_pose, guass model, partial number
    void initial_pose() {

    }


    // update partial pose with movement model
    // given robot movement, gauss model
    void update_pose() {

    }

    // update weights using laser scan and map
    void update_match_weight() {

    }

    // update weight using map transformation
    void update_map() {

    }

    // mormalise weight
    void normalise() {

    }

    // get best partial
    void get_best_pose() {

    }

    // send robot pose
    void send_pose() {

    }

    // publish map
    void send_map() {

    }

    // main loop
    void run() {

        // wait a signal

        // 1. get laser scan

        // 2. update partial pose from tf tree or lastest partial pose or

        // 3. compute match weight

        // 4. update map

        // 5. find best weight

        // 6. update pose and map


    }
};

struct ALut {
    float m_a01;
    float m_a11;
    float m_resolution;
    float m_scale;
    size_t m_size;
    std::vector<float> m_data;
    float *data;

    ALut(float a01, float a10, float resolution) : m_a01(a01), m_a11(1 - a10),
                                                   m_resolution(resolution),
                                                   m_scale(1 / resolution), m_size(m_scale), m_data(m_size) {


        for (float i = 0; i < m_scale; i++) {
            float q = i * m_resolution;
            m_data[i] = (1 - q) * m_a01 + q * m_a11;
        }
        data = &(m_data[0]);
    }

    float operator()(float q) {
        size_t index = q * m_scale;
        return m_data[index];
    }


};

struct BLut {
    float m_b00;
    float m_b11;
    float m_resolution;
    float m_scale;
    size_t m_size;
    std::vector<float> m_data;
    float *data;

    BLut(float b00, float b11, float resolution) : m_b00(b00), m_b11(b11),
                                                   m_resolution(resolution),
                                                   m_scale(1 / resolution), m_size(m_scale), m_data(m_size) {

        for (float i = 0; i < m_scale; i++) {
            float q = i * m_resolution;
            m_data[i] = q * m_b11 / ((1 - q) * m_b00 + q * m_b11);
        }
        data = &(m_data[0]);
    }

    float operator()(float q) {
        size_t index = q * m_scale;
        return m_data[index];
    }
};

void ff(float a, float b, std::vector<float> &vc) {

    float p = pow(a, b);
    if (0 == p || std::isnan(p)) {
        ff(a, int(b * 0.5), vc);
        ff(a, b - int(b * 0.5), vc);

    } else {
        vc.push_back(b);
    }
}

void prod_recu(mp_util::mp_float &res, float b, float e) {

    std::vector<float> vc(0);
    ff(b, e, vc);
    for (auto i : vc) {
        res.prod_v(pow(b, i));
    }
}

int main(int argc, char **argv) {

    std::cout << "hello" << std::endl;









    // copute 0.3^30 * 0.6^45
#if 0

    {
        float a = 0.3, af = 45;
        float b = 0.6, bf = 66;
        std::cout << pow(a,100) << std::endl;

        auto w = mp_util::mp_pow(0.1,100);
        auto w2 = mp_util::mp_pow(0.1,100);


        mp_util::mp_float ss = 0.0;
        for(int i = 0; i < 800; i++){
            auto d1 = mp_util::mp_pow(0.1,100);
            auto  d2 = mp_util::mp_pow(0.1,100);

//            d1.prod_v(i);
            d1.prod_mpf(d2);
            ss.add_mpf(d1);


        }


        std::cout << "ss: ";
        ss.print();
        w.prod_mpf(w2);
        w.devide_mpf(ss);
        std::cout << "w: ";
        w.print();
//        w.prod_v(2.5);
//        w.prod_mpf(w2);

        w2.prod_mpf(w);

        std::cout << "w: ";

        w.print();
        std::cout << "w2: ";

        w2.print();
        ss.add_mpf(w2);


    }

    exit(3);
#endif




    GaussLUT glt(0.0, 0.5, 0.6, 100);

    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan2", 1);
    ros::Publisher scan_pub0 = nh.advertise<sensor_msgs::LaserScan>("scan0", 1);

    ros_util::Node node;
    auto scan_ptr = node.createSubscriber<sensor_msgs::LaserScan>("/scan", 1);
    nav_msgs::OccupancyGrid m_map;
    m_map.header.frame_id = "/map";
    m_map.header.stamp = ros::Time::now();
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    geometry_msgs::PoseWithCovarianceStamped initi_pose;
    geometry_msgs::PoseArray partial_cloud;

    auto initialpose_ptr = node.createSubscriber<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Publisher partial_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("/partial_cloud", 1);
    ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    geometry_msgs::PoseStamped final_pose;

    partial_cloud.header.frame_id = "base_laser";
    final_pose.header.frame_id = "base_laser";


//    test_main();
//    exit(3);

    //todo: test Q, A, B
    // Hmm data
    //========================
    // A cell value
    // A0 free
    float freeA01 = 0.2;
    float freeA11 = 0.6;
    ALut freeAlut(freeA01, 1 - freeA11, 0.001);


    // A1 occu
    float occuA01 = 0.4;
    float occuA11 = 0.8;
    ALut occuAlut(occuA01, 1 - occuA11, 0.001);

    // A2 dynamic

    float dynaA01 = 0.4;
    float dynaA11 = 0.6;
    ALut dynaAlut(dynaA01, 1 - dynaA11, 0.001);


    //========================
    // B value
    float B00 = 0.6;
    float B01 = 0.35;

    float B10 = 0.35;
    float B11 = 0.6;

    BLut missBlut(B00, B01, 0.001);
    BLut hitBlut(B10, B11, 0.001);

    // todo: change_prob
    // free 0-0,0-1,1-1,1-0
    // occu 0-0,0-1,1-1,1-0
    // dyna 0-0,0-1,1-1,1-0
    std::valarray<float> change_prob(12);
    change_prob[0] = 1 - freeA01;
    change_prob[1] = freeA01;
    change_prob[2] = freeA11;
    change_prob[3] = 1 - freeA11;
    change_prob[4] = 1 - occuA01;
    change_prob[5] = occuA01;
    change_prob[6] = occuA11;
    change_prob[7] = 1 - occuA11;
    change_prob[8] = 1 - dynaA01;
    change_prob[9] = dynaA01;
    change_prob[10] = dynaA11;
    change_prob[11] = 1 - dynaA11;

    float bypass_free_q = 0.033719;
    float limit_free_q = 0.2112;
#if 0

    {

        // condition 1
        // free cell get hit, q --> 0.55
        // free cell get miss, q --> 0.1

        //Q = Q * A * B
        double w = 1.0;

        std::cout << "free cell get hit, q --> 0.55\n";
        float q = 0.5;
        for(int i = 0;i < 20; i++){

            q = hitBlut(q);

            std::cout << q << ",";
            q = freeAlut(q);

            std::cout << q << "\n";

        }
        std::cout << "free cell get miss, q --> 0.1\n";

        for(int i = 0;i < 20; i++){

            q = missBlut(q);

            std::cout << q << ",";
            q = freeAlut(q);

            std::cout << q << "\n";

        }
        std::cout << "occu cell get miss, q --> 0.45\n";

        q = 0.5;
        for(int i = 0;i < 20; i++){

            q = missBlut(q);

            std::cout << q << ",";
            q = occuAlut(q);

            std::cout << q << "\n";

        }
        std::cout << "occu cell get hit, q --> 0.9\n";
        w = 1.0;

        for(int i = 0;i < 20; i++){

            q = hitBlut(q);

            std::cout << q << ",";
            q = occuAlut(q);

            std::cout << q << "\n";
            w *= q;

        }
        std::cout << "w: " << w << "\n";

        std::cout << "dyna cell get hit, q --> 0.55\n";

        q = 0.5;
        w = 1.0;
        for(int i = 0;i < 20; i++){

            q = hitBlut(q);

            std::cout << q << ",";
            q = dynaAlut(q);
            q = dynaAlut(q);

            std::cout << q << "\n";
            w *= q;

        }
        std::cout << "w: " << w << "\n";
        std::cout << "dyna cell get miss, q --> 0.45\n";

        q = 0.5;
        for(int i = 0;i < 20; i++){

            q = missBlut(q);

            std::cout << q << ",";
            q = dynaAlut(q);
            q = dynaAlut(q);

            std::cout << q << "\n";

        }
        exit(9);
    }
#endif



    ublas_util::Transformation2d trans(0.5, 0.2, 0.5 * M_PI);


    // test
    // 1. laserscan
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "base_laser";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    int scan_num = 180 * 4;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (scan_num);
    scan.range_max = 30;
    scan.range_min = 0.0;
    boost::random::mt19937 rng;         // produces randomness out of thin air

    boost::random::mt19937 rng_x;         // produces randomness out of thin air
    boost::random::mt19937 rng_y;         // produces randomness out of thin air
    boost::random::mt19937 rng_yaw;         // produces randomness out of thin air

    // see pseudo-random number generators
    boost::random::uniform_real_distribution<> six(6.0, 6.2);


    boost::random::normal_distribution<> gen_x(0.0, 0.35);
    boost::random::normal_distribution<> gen_y(0.0, 0.35);
    boost::random::normal_distribution<> gen_yaw(0.0, 0.13);


    // distribution that maps to 1..6
    // see random number distributions

    for (int i = 0; i < scan_num; i++) {
//        int x = six(rng);

        scan.ranges.push_back(six(rng));
    }




    //===========
    // create map
    float map_resolution = 0.1;
    float map_resolution_scale = 1.0 / map_resolution;

    grid_util::LaserGrid laser_grid(0.05);

    ros_util::LaserScan m_scan;
    // 3. map grid
    grid_util::MapGrid map_grid(map_resolution, 30.0);
    map_grid.setOrigin(-15.0, -15.0, 0.0);
    auto map_origin = map_grid.getgetOriginMatrix_ublas();
    ublas_util::Transformation2d map_laser(0, 0, 0.0 * M_PI);
    cv::Mat mat(int(30 / map_resolution), int(30 / map_resolution), CV_8UC1, cv::Scalar(100));
    cv::Mat mat_update(int(30 / map_resolution), int(30 / map_resolution), CV_8UC1, cv::Scalar(100));

    laser_grid.setContour_Offset(0.05);
    auto map_origin_inverse = map_origin.inverse();
    ublas::matrix<float> minv = ublas::prod(map_origin_inverse, map_laser.matrix());
    mp_util::mp_float global_best_weight = 0.0;


    std::cout << "start record laser scan" << std::endl;
    for (int i = 0; i < 40; i++) {

        node.getOneMsg("/scan", -1);
        scan = *(scan_ptr);
        for (auto &reading : scan.ranges) {
            reading = std::clip(reading, 0.0f, 10.0f);
        }
        // 2. laser grid
        laser_grid.update(scan);
        auto fp = laser_grid.getFreePointsMat_ublas();
        auto op = laser_grid.getOccuPointsMat_ublas();

        auto fp1 = fp;
        auto op1 = op;
        ublas::noalias(fp1) = ublas::prod(minv, fp);
        ublas::noalias(op1) = ublas::prod(minv, op);


        for (size_t j = 0; j < fp1.size2(); j++) {
            cv::Point p(int(fp1(1, j) * map_resolution_scale), int(fp1(0, j) * map_resolution_scale));
            p.x = std::clip(p.x, 0, mat.cols);
            p.y = std::clip(p.y, 0, mat.rows);
            mat.at<uchar>(p) = 0;

        }
        for (size_t j = 0; j < op1.size2(); j++) {
            cv::Point p(int(op1(1, j) * map_resolution_scale), int(op1(0, j) * map_resolution_scale));

            p.x = std::clip(p.x, 0, mat.cols);

            p.y = std::clip(p.y, 0, mat.rows);
            mat.at<uchar>(p) = 255;

        }
        std::cout << "start fix laser scan" << std::endl;

        mat(cv::Range(220 / 2, 365 / 2), cv::Range(220 / 2, 290 / 2)) = cv::Scalar(0);

        cv::line(mat, cv::Point(220 / 2, 220 / 2), cv::Point(220 / 2, 365 / 2), cv::Scalar(255));
        cv::line(mat, cv::Point(220 / 2, 220 / 2), cv::Point(330 / 2, 220 / 2), cv::Scalar(255));
        cv::line(mat, cv::Point(220 / 2, 365 / 2), cv::Point(270 / 2, 365 / 2), cv::Scalar(255));

        mat(cv::Range(230 / 2, 350 / 2), cv::Range(230 / 2, 380 / 2)) = cv::Scalar(150);

        cv::imshow("laser", mat);

        cv::waitKey(1);

    }

//    cv::imwrite("map.jpg",mat);
//    mat = cv::imread("map.jpg", -1);
#if 1
    std::cout << "wait intial pose" << std::endl;

    m_map.info.width = mat.cols;
    m_map.info.height = mat.rows;
    m_map.info.resolution = map_resolution;
    m_map.data.clear();
    m_map.info.origin.position.x = -15;
    m_map.info.origin.position.y = -15;
    m_map.info.origin.orientation.w = 1;
    for (int i = m_map.info.width - 1; i >= 0; i--) {
        for (size_t j = 0; j < m_map.info.height; j++) {
            signed int d = mat.at<uchar>(i, j);
            m_map.data.emplace_back(d);
        }
    }

#endif

    // test map update
    {
        std::cout << "start update map " << std::endl;

        //create one partial
        std::vector<Partial> Partials(100);
        Partials[0].minv_beam = ublas::identity_matrix<float>(3, 3);
        Partials[0].minv_beam(0, 2) = 10;
        Partials[0].minv_beam(1, 2) = 10;

        boost::random::normal_distribution<> gen_x(0.0, 0.15);
        boost::random::normal_distribution<> gen_y(0.0, 0.15);
        boost::random::normal_distribution<> gen_yaw(0.0, 0.15);

        Partials[0].minv_beam = minv;


        //global data
        // store cell type
        // todo:create matrix from map
        // todo: change matrix resolution
        // map sesolution 0.05 --> stste resolution 0.05 * 2
        float state_map_resolution = 2;
        float state_map_scale = 10.0;
        ublas::compressed_matrix<char> global_map_cell(300, 300, 300 * 300);
        for (int i = 0; i < mat.rows; i++) {
            for (int j = 0; j < mat.cols; j++) {
                if (mat.at<uchar>(i, j) == 150 || mat.at<uchar>(i, j) == 100) {
                    global_map_cell(int(i), int(j)) = 2;

                }

            }
        }
        for (int i = 0; i < mat.rows; i++) {
            for (int j = 0; j < mat.cols; j++) {

                if (mat.at<uchar>(i, j) == 255) {

                    global_map_cell(int(i), int(j)) = 1;
                }
            }
        }

        std::cout << "start update 22 " << std::endl;

        std::array<float, 3> cell_q = {0.333, 0.666, 0.5};



        std::cout << "start loop " << std::endl;
        bool g = node.getOneMsg("/initialpose", -0.1);
        auto initialpose = *initialpose_ptr;

        if (g) {
#pragma omp simd
            for (int i = 0; i < 100; i++) {

                ublas_util::Transformation2d map_laser_beam(gen_x(rng), gen_y(rng), gen_yaw(rng));

                ublas::matrix<float> minv_beam(3, 3);
                geometry_msgs::Pose p;

                p.position.x = initialpose.pose.pose.position.x + map_laser_beam.getX();

                p.position.y = initialpose.pose.pose.position.y + map_laser_beam.getY();

                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(
                        tf::getYaw(initialpose.pose.pose.orientation) + map_laser_beam.getYaw()), p.orientation);

                map_laser_beam.set(p.position.x, p.position.y, tf::getYaw(p.orientation));
                if (i == 199) {
                    map_laser_beam.set(0.02, 0.02, 0.003);
                }
                Partials[i].map_laser_pose = map_laser_beam;

                partial_cloud.poses.push_back(p);

                ublas::noalias(minv_beam) = ublas::prod(map_origin_inverse, map_laser_beam.matrix());

                // todo: use sample pose
                Partials[i].minv_beam = minv_beam; //minv; //
                Partials[i].set_m_change_cell_prob(change_prob);
            }

            partial_cloud.header.stamp = ros::Time::now();
            partial_cloud_pub.publish(partial_cloud);
        }

        int loop_cnt = 0;

        while (ros::ok()) {
            bool g = node.getOneMsg("/initialpose", 0.01);






//            Partials[0].minv_beam = minv; //minv; //

            // get laser
            node.getOneMsg("/scan", -1);
            scan = *(scan_ptr);
            for (auto &reading : scan.ranges) {
                reading = std::clip(reading, 0.0f, 10.0f);
            }
            // 2. laser grid
            laser_grid.update(scan);

            auto fp = laser_grid.getFreePointsMat_ublas();
            auto op = laser_grid.getOccuPointsMat_ublas();


            mat_update = cv::Scalar(100);

            // process scan
            time_util::Timer t1;
            t1.start();

            tbb::parallel_for(tbb::blocked_range<int>(0, 100),
                              [&](tbb::blocked_range<int> r) {

//#pragma omp simd
                                  for (auto it = r.begin(); it < r.end(); it++) {
                                      // get copy of local map
                                      ublas::compressed_matrix<float> &global_map = Partials[it].global_map;
                                      ublas::compressed_matrix<float> &old_global_map = Partials[it].old_global_map;

                                      ublas::compressed_matrix<float> &current_global_map = Partials[it].current_global_map;
                                      current_global_map = global_map;
                                      ublas::matrix<float> op_beam = op;
                                      ublas::noalias(op_beam) = ublas::prod(Partials[it].minv_beam, op);

                                      ublas::matrix<float> fp_beam = fp;
                                      ublas::noalias(fp_beam) = ublas::prod(Partials[it].minv_beam, fp);


                                      // ================================
                                      // Update Miss cell

                                      size_t fp_sz = fp.size2();

                                      //todo: update vector size or not
                                      if (fp_sz > Partials[it].freePointQ.size()) {
                                          Partials[it].freePointIndex.resize(fp_sz * 2);
                                          Partials[it].freePointQ.resize(fp_sz);
                                          Partials[it].freePointQ_.resize(fp_sz);
                                      }
                                      size_t freeCache_sz = 0.5 * Partials[it].freePointIndex.size();


                                      //get pointer
                                      float *freeQPtr = &(Partials[it].freePointQ[0]);
                                      float *freeQ_Ptr = &(Partials[it].freePointQ_[0]);
                                      int *freeIndex1_ptr = &(Partials[it].freePointIndex[0]);
                                      int *freeIndex2_ptr = freeIndex1_ptr +
                                                            freeCache_sz;// &(Partials[it].freePointIndex[0.5 * freeCache_sz]);

                                      float *fp_beam_ptr = fp_beam.data().begin();

                                      int updateFp_cnt = 0;
                                      for (int i = 0; i < fp_sz; i++) {
                                          int px = int((fp_beam_ptr[i] * map_resolution_scale));
                                          int py = int((fp_beam_ptr[i + fp_sz] * map_resolution_scale));
                                          // get q from cache map
                                          float q = current_global_map(px, py);

//                                          std::cout << "q: " << q << std::endl;
                                          // check q value 1. q = 0, not valid , get default value from global map
                                          if (q == 0.0) {
                                              q = cell_q[int(global_map_cell(px, py))];
//                                              std::cout << "px: " << px << ", py: " << py << std::endl;
//                                              std::cout << "update q: " << q <<"," << int(global_map_cell(200, 200))<<std::endl;
//                                              old_global_map(px, py) = q;

                                          }
//                                          std::cout << "i = " << i <<", q1 = " << q << std::endl;
//                                          mat_update.at<char>(px, py) = 255;

//                                          // skip update rule
//                                          if (q > 0.0) {
//                                              freeQPtr[updateFp_cnt] = q;
//                                              freeIndex1_ptr[updateFp_cnt] = px;
//                                              freeIndex2_ptr[updateFp_cnt] = py;
//                                              freeQ_Ptr[updateFp_cnt] = 1 - q;
//                                              updateFp_cnt++;
//
//                                          }


                                          freeQPtr[i] = q;
                                          freeIndex1_ptr[i] = px;
                                          freeIndex2_ptr[i] = py;
                                          freeQ_Ptr[i] = 1 - q;
                                      }

                                      //compute Q = Q * B

                                      for (int i = 0; i < fp_sz; i++) {
                                          float q1 = freeQPtr[i] * B01;
//                                          std::cout << "i = " << i <<", q1 = " << q1 << std::endl;
                                          freeQPtr[i] = q1 / (freeQ_Ptr[i] * B00 + q1);

//                                          std::cout << "i = " << i <<", freeQPtr[i] = " << freeQPtr[i] << std::endl;

                                      }
                                      // update global map
                                      for (int i = 0; i < fp_sz; i++) {

                                          global_map(freeIndex1_ptr[i], freeIndex2_ptr[i]) = freeQPtr[i];
//                                          mat_update.at<char>(freeIndex1_ptr[i], freeIndex2_ptr[i]) = 0;

                                      }



//
// Update Hit cell

                                      size_t op_sz = op.size2();

                                      // resize vector
                                      if (op_sz > Partials[it].occuPointQ.size()) {
                                          Partials[it].occuPointIndex.resize(op_sz * 2);
                                          Partials[it].occuPointQ.resize(op_sz);
                                          Partials[it].occuPointQ_.resize(op_sz);

                                      }

                                      size_t occuCache_sz = 0.5 * Partials[it].occuPointIndex.size();

                                      //get pointer
                                      float *occuQPtr = &(Partials[it].occuPointQ[0]);
                                      float *occuQ_Ptr = &(Partials[it].occuPointQ_[0]);
                                      int *occuIndex1_ptr = &(Partials[it].occuPointIndex[0]);
                                      int *occuIndex2_ptr = occuIndex1_ptr +
                                                            occuCache_sz;// &(Partials[it].occuPointIndex[0.5 * occuCache_sz]);

                                      float *op_beam_ptr = op_beam.data().begin();


                                      for (int i = 0; i < op_sz; i++) {
                                          // get q from cache map
                                          int px = int((op_beam_ptr[i] * map_resolution_scale));
                                          int py = int((op_beam_ptr[i + op_sz] * map_resolution_scale));
//                                          int(round( op_beam_ptr[i]* 20.0f));
                                          float q = current_global_map(px, py);

                                          // check q value 1. q = 0, not valid , get default value from global map
                                          if (q == 0) {
                                              q = cell_q[global_map_cell(px, py)];
//                                              old_global_map(px, py) = q;

                                          }

                                          occuQPtr[i] = q;
                                          occuQ_Ptr[i] = 1 - q;

                                          occuIndex1_ptr[i] = px;
                                          occuIndex2_ptr[i] = py;

//                                          Partials[it].occuPointIndex[i] = px;
//                                          Partials[it].occuPointIndex[i + size_t(0.5*occuCache_sz)] = py;


//                                          std::cout << occuIndex1_ptr[i] <<"," << occuIndex2_ptr[i] << ",";

//                                          std::cout << px <<",";
//                                          mat_update.at<char>(px, py) = 255;



                                      }

                                      // compute Q


#pragma omp simd
                                      for (int i = 0; i < op_sz; i++) {
                                          float q1 = occuQPtr[i] * B11;


                                          occuQPtr[i] = q1 / (occuQ_Ptr[i] * B10 + q1);

                                      }

                                      for (int i = 0; i < op_sz; i++) {

                                          global_map(occuIndex1_ptr[i], occuIndex2_ptr[i]) = occuQPtr[i];
//                                          mat_update.at<char>(occuIndex1_ptr[i], occuIndex2_ptr[i]) = 255;

                                      }






                                      // =================
                                      // Q = Q * A

//                                      std::valarray<int> erase_index1(1000 * 2);
//                                      std::valarray<int> erase_index2(1000 * 2);
//                                      Partials[it].erase_index1.resize(2000);
//                                      Partials[it].erase_index2.resize(2000);
                                      int *erase_index1_ptr = &(Partials[it].erase_index1[0]);
                                      int *erase_index2_ptr = &(Partials[it].erase_index2[0]);

                                      int remove_cnt = 0;


                                      Partials[it].reset_m_chnage_cell_cnt();

//                                      int *cell_change_cnt_ptr = &(Partials[it].m_chnage_cell_cnt[0]);
                                      // there may be too many point
                                      // how to avoid useless computation

#if 1

                                      if (loop_cnt == 0) {
//                                          old_global_map = global_map;
                                          loop_cnt++;

                                      } else {
                                          old_global_map = current_global_map;

                                      }
                                      current_global_map = global_map;

                                      float forward_prob[20 * 20 * 30 * 30];
                                      int forward_prob_cnt = 0;

                                      // 0.5 - 1.0 --> 0 - 100
                                      std::vector<int> forward_prob_lut(50, 0);
                                      int *forward_prob_lut_ptr = &(Partials[it].m_chnage_cell_cnt[0]);

                                      float forward_prob_lut_scale = 100.0;

                                      for (auto it1 = global_map.begin1(); it1 != global_map.end1(); it1++) {
                                          for (auto it2 = it1.begin(); it2 != it1.end(); it2++) {
                                              float old_q = *it2;
                                              float update_q = 0;
                                              int cell_type = global_map_cell(it2.index1(), it2.index2());

//                                              std::cout << "cell_type " << cell_type << " , q = " << old_q << std::endl;
                                              // update
                                              if (0 == cell_type) {
#if 0
                                                  if (old_q - round(old_q) < bypass_free_q)
                                                      *it2 = limit_free_q;
                                                  else
                                                      *it2 = (1 - old_q) * freeA01 + old_q * freeA11;

#endif


                                                  *it2 = (1 - old_q) * freeA01 + old_q * freeA11;

//                                                  std::cout << *it2 << "|";
                                                  if (old_global_map(it2.index1(), it2.index2()) <= 0.5) {
                                                      // count ++
                                                      if (*it2 <= 0.5) {
                                                          // 0-0
//                                                          cell_change_cnt_ptr[0]++;
//                                                          forward_prob_lut_ptr[int(forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      } else {
                                                          // 0-1
//                                                          cell_change_cnt_ptr[1]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  } else {
                                                      // count ++
                                                      if (*it2 <= 0.5) {
                                                          // 1-0
//                                                          cell_change_cnt_ptr[3]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      } else {
                                                          // 1-1
//                                                          cell_change_cnt_ptr[2]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  }

                                              } else if (1 == cell_type) {


                                                  *it2 = (1 - old_q) * occuA01 + old_q * occuA11;
                                                  if (old_global_map(it2.index1(), it2.index2()) <= 0.5) {
                                                      // count ++
                                                      if (*it2 <= 0.5) {
                                                          // 0-0
//                                                          cell_change_cnt_ptr[4]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      } else {
                                                          // 0-1
//                                                          cell_change_cnt_ptr[5]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  } else {
                                                      // count ++
                                                      if (*it2 <= 0.5) {
                                                          // 1-0
//                                                          cell_change_cnt_ptr[7]++;

                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;
                                                      } else {
                                                          // 1-1
//                                                          cell_change_cnt_ptr[6]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  }

                                              } else if (2 == cell_type) {

                                                  *it2 = (1 - old_q) * dynaA01 + old_q * dynaA11;
//                                                  std::cout << "~ "<< *it2 << std::endl;

                                                  if (old_global_map(it2.index1(), it2.index2()) <= 0.5) {
                                                      // count ++
                                                      if (*it2 <= 0.5) {
                                                          // 0-0
//                                                          cell_change_cnt_ptr[8]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      } else {
                                                          // 0-1
//                                                          cell_change_cnt_ptr[9]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  } else {
                                                      // count ++
                                                      if (*it2 < 0.5) {
                                                          // 1-0
//                                                          cell_change_cnt_ptr[11]++;
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      } else {
//                                                          cell_change_cnt_ptr[10]++;
                                                          // 1-1
                                                          forward_prob_lut_ptr[int(
                                                                  forward_prob_lut_scale * (fabs(0.5 - old_q)))] += 1;

                                                      }
                                                  }
                                              }

#if 0
                                              if (0 == old_q){
                                                      *it2 = freeAlut(old_q);
                                                  }else if(1 == old_q){
                                                      *it2 = occuAlut(old_q);
                                                  }else if(2 == old_q){
                                                      *it2 = dynaAlut(old_q);
                                                  }
#endif

                                              // count chnage




                                              // erase
                                              // if not update in Q = Q * B
                                              // then it may be unchanged
#if 1
//                                              std::cout << "\nerase:\n";
//                                              std::cout << *it2 << "," << old_q << it2.index1() << "," <<it2.index2() << std::endl;

                                              if (remove_cnt < 1999 && fabs(*it2 - old_q) < 0.001) {

                                                  erase_index1_ptr[remove_cnt] = it2.index1();

                                                  erase_index2_ptr[remove_cnt] = it2.index2();
//                                                  std::cout << "==erase: " <<*it2 << "," << old_q << it2.index1() << "," <<it2.index2() << std::endl;

                                                  remove_cnt++;
                                              }
//                                              std::cout << "\nerase:\n" << std::endl;


#endif
                                              forward_prob_cnt++;

                                          }
                                      }
                                      //================ end loop through map
                                      for (int i = 0; i < forward_prob_cnt; i++) {

                                      }

#if 0
                                      for (int i = 0; i < 2000; i++) {

                                          global_map.erase_element(erase_index1_ptr[i],erase_index2_ptr[i]);

                                      }
#endif

#endif


                                  }
                              });


            t1.stop();

            std::cout << "loop done  time: " << t1.elapsedSeconds() << std::endl;
////////



/////////
            /*
             * how to define update prob
             * cell count shoudn't be too big
             *
             *  how to compute 0.2^34 * 0.6^123
             *
             *
             *
             * */


            auto min_change_cnt = Partials[0].m_chnage_cell_cnt;


            // find max weight
            // find min exp factor
            float min_change_prob = Partials[0].m_normal_change_cell_prob;
            int best_id = 0;

            std::cout << "\n====check cnt";
            for (int i = 0; i < Partials.size(); i++) {

                for (int j = 0; j < 50; j++) {
                    if (min_change_cnt[j] > Partials[i].m_chnage_cell_cnt[j]) {
                        min_change_cnt[j] = Partials[i].m_chnage_cell_cnt[j];
                    }
                }

            }
            std::cout << "min_change_cnt" << std::endl;

            for (auto i : min_change_cnt) {
                std::cout << i << ",";
            }
            std::cout << std::endl;

            for (int i = 0; i < Partials.size(); i++) {

                for (int j = 0; j < 50; j++) {
                    Partials[i].m_chnage_cell_cnt[j] -= min_change_cnt[j];
                }

            }
//            std::cout << "\n check map update weight:\n";
            mp_util::mp_float allweight = 0.0;
            for (int i = 0; i < Partials.size(); i++) {


                mp_util::mp_float &partail_weight = Partials[i].map_weight;
//                partail_weight = 1.0;
                for (int j = 0; j < 50; j++) {
                    auto exp = Partials[i].m_chnage_cell_cnt[j];
                    if (exp != 0) {
                        auto p = mp_util::mp_pow(0.5 + 0.01 * j, exp);
                        partail_weight.prod_mpf(p);
                    }
//                    std::cout << Partials[i].m_chnage_cell_cnt[j]<< ",";
                }
                allweight.add_mpf(partail_weight);
            }

            std::cout << "\n======all weight: ";
            allweight.print();
            std::cout << "\n======partial weight: ";

            for (int i = 0; i < Partials.size(); i++) {


                mp_util::mp_float &partail_weight = Partials[i].map_weight;
                partail_weight.devide_mpf(allweight);
                std::cout << "\ni:" << i << ", ";
                partail_weight.print();
                std::cout << ",m_chnage_cell_cnt: ";

                for (int j = 0; j < 50; j++) {
                    std::cout << Partials[i].m_chnage_cell_cnt[j] << ", ";
                }
                std::cout << "\n";

            }

            best_id = 0;
            auto best_weight = Partials[0].map_weight;
            for (int i = 0; i < Partials.size(); i++) {
                if (Partials[i].map_weight > best_weight) {
                    best_weight = Partials[i].map_weight;
                    best_id = i;
                }
            }

            std::cout << "best id :" << best_id;
            std::cout << "pose:\nx: " << Partials[best_id].map_laser_pose.getX() << " y: "
                      << Partials[best_id].map_laser_pose.getY()
                      << " yaw: " << Partials[best_id].map_laser_pose.getYaw() << std::endl;

            std::cout << "best weight: ";
            Partials[best_id].map_weight.print();
            if (1 || Partials[best_id].map_weight > global_best_weight) {
                global_best_weight = Partials[best_id].map_weight;
                //global_enable_match = true;
                final_pose.pose.position.x = Partials[best_id].map_laser_pose.getX();
                final_pose.pose.position.y = Partials[best_id].map_laser_pose.getY();

                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(Partials[best_id].map_laser_pose.getYaw()),
                                      final_pose.pose.orientation);

            } else {
                // global_enable_match = false;

            }
            final_pose.header.stamp = ros::Time::now();
//        cv::imshow("laser", mat);
//        cv::imshow("laser", mat);

//        cv::imshow("mat_dynamic", mat_dynamic);
            final_pose_pub.publish(final_pose);
            (*initialpose_ptr).pose.pose = final_pose.pose;

            //            mat = cv::Scalar(0);
            ublas::compressed_matrix<float> &global_map = Partials[best_id].global_map;

            for (auto it1 = global_map.begin1(); it1 != global_map.end1(); it1++) {
                for (auto it2 = it1.begin(); it2 != it1.end(); it2++) {

//                    std::cout << "q = " << *it2 << ",[ " << it2.index1() << ", " << it2.index2() << std::endl;

                    if (*it2 <= 0.5) {
                        mat_update.at<char>(it2.index1(), it2.index2()) = 0;
                    }
                    if (*it2 > 0.5) {
                        mat_update.at<char>(it2.index1(), it2.index2()) = 255;
                    }
                }
            }


            m_map.data.clear();
            for (int i = m_map.info.width - 1; i >= 0; i--) {
                for (size_t j = 0; j < m_map.info.height; j++) {
                    signed int d = mat.at<uchar>(i, j);
                    auto e = d;
//                    std::cout << "idx: " << i << ", " << j << " = " << d << std::endl;
                    if (d == 0) {
                        e = 0;
                    } else if (d == 100) {
                        e = -1;
                    } else if (d == 255) {
                        e = 100;
                    }
                    if (d == 150) {
                        e = 0;
                    }

                    signed int dl = mat_update.at<uchar>(i, j);
                    if (dl == 0)
                        e = 0;
                    if (dl == 255)
                        e = 100;

                    m_map.data.emplace_back(e);
                }
            }
            m_map.header.stamp = ros::Time::now();

            map_pub.publish(m_map);

//            exit(0);
            cv::waitKey(1);

        }

    }




//    std::cout << "get data\n" << m_scan.getXsYsMatrix_ublas() << std::endl;
    node.getOneMsg("/initialpose", -1);

    std::vector<float> js(40);
    std::iota(js.begin(), js.end(), -20);

//    bool global_enable_match = true;
    time_util::Timer timer;
//    global_enable_match = false;
    while (true) {

#if 0
        if (global_enable_match){
            bool g = node.getOneMsg("/initialpose", 0.1);
            if (g){
                global_enable_match = true;

            }

        }else{
            bool g = node.getOneMsg("/initialpose", 0.1);
            if (g){
                global_enable_match = true;

            }
            continue;
        }
#endif

        bool g = node.getOneMsg("/initialpose", 0.1);
        if (g) {
            global_best_weight = 0.0;
        }
        auto initialpose = *initialpose_ptr;


        timer.start();
        std::vector<Partial> Partials(1000);


        //todo:create partial
#pragma omp simd
        for (int i = 0; i < 1000; i++) {
            ublas_util::Transformation2d map_laser_beam(gen_x(rng), gen_y(rng), gen_yaw(rng));
            ublas::matrix<float> minv_beam(3, 3);
            geometry_msgs::Pose p;

            p.position.x = initialpose.pose.pose.position.x + map_laser_beam.getX();

            p.position.y = initialpose.pose.pose.position.y + map_laser_beam.getY();

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(
                    tf::getYaw(initialpose.pose.pose.orientation) + map_laser_beam.getYaw()), p.orientation);

            map_laser_beam.set(p.position.x, p.position.y, tf::getYaw(p.orientation));

            Partials[i].map_laser_pose = map_laser_beam;

            partial_cloud.poses.push_back(p);

            ublas::noalias(minv_beam) = ublas::prod(map_origin_inverse, map_laser_beam.matrix());

            Partials[i].minv_beam = minv_beam;
        }
        timer.stop();
        std::cout << "sample time: " << timer.elapsedSeconds() << std::endl;
#if 0
        { int best_id = 0;
            std::cout << "best id :" << best_id;
            std::cout << "pose:\nx: " << Partials[best_id].map_laser_pose.getX() << " y: "
                      << Partials[best_id].map_laser_pose.getY()
                      << " yaw: " << Partials[best_id].map_laser_pose.getYaw() << std::endl;

            exit(8);
        }

#endif

        partial_cloud.header.stamp = ros::Time::now();
        partial_cloud_pub.publish(partial_cloud);



//    std::cout << "get data\n" << laser_grid.getFreePointsMat_ublas() << std::endl;
//    std::cout << "get data\n" << laser_grid.getOccuPointsMat_ublas() << std::endl;

        auto fp = laser_grid.getFreePointsMat_ublas();
//        auto fp2 = laser_grid.getFreePointsMat();
        auto op = laser_grid.getOccuPointsMat_ublas();
        // check cv mat
        cv::Mat mat_dynamic(400, 400, CV_8UC1, cv::Scalar(0));
        cv::Mat mat_pritial(400, 400, CV_8UC1, cv::Scalar(0));





#if 0
        eigen_util::TransformationMatrix2d<float> map_laser2(10,10,0.6*M_PI);
    fp2 = map_laser2*fp2;
    std::cout << "map laser:\n" << map_laser.matrix() << std::endl;
    std::cout << "map laser2:\n" << map_laser2.matrix() << std::endl;

    auto fp1 = fp;
    auto op1 = op;
//    ublas::inverse(map_laser2,map_laser2);


    std::cout << "map laser inverse:\n" << map_laser.inverse() << std::endl;
    std::cout << "map laser2 inverse:\n" << map_laser2.matrix().inverse() << std::endl;
    std::cout << "minv:\n" << minv << std::endl;
    ublas::noalias(fp1) = ublas::prod(minv, fp);
    ublas::noalias(op1) = ublas::prod(minv, op);







    // check dynamic type
    // create Q matrix for each type [Q0, Q1, Q2...]
    // compute A*Q

    ublas::matrix<float> A1(2,2);

    std::vector<size_t> freePointsXs;
    std::vector<size_t> freePointsYs;
    std::vector<float> freePointsQs;


//    for (size_t i = 0 ; i < op1.size2(); i++){
//        cv::Point p(round(op1(1,i)/0.05), round(op1(0,i)/0.05));
//        mat.at<uchar>(p) = 255;
//
//    }

    // create Q matrix from freePointsQs
    ublas::matrix<float> freeQ(2,100);

    auto freeQ2 = freeQ;
#endif



//    ublas::noalias(freeQ2) = ublas::prod(A, freeQ);

        // update cache


        // create tempral sparase matrix for query
        // update matrix



        // ray trace
        // get beam and index
//    laser_grid.createBeam();
//    auto beams = laser_grid.getBeam();

        std::cout << "================" << std::endl;

        timer.start();





        //==============
        auto scan2 = scan;
        size_t sz = laser_grid.getScan().ranges.size();

//    std::valarray<float> scan3(0.0, sz);

//    std::cout << "op1: \n" << op1 << std::endl;
//    std::cout << "beams: \n" << beams << std::endl;
//
//    exit(2);


        float max_range = laser_grid.getScan().range_max;
        auto laser_ranges = laser_grid.getScan().ranges;
        auto laser_resolution = laser_grid.getResolution();
        float mat_dynamic_cols = float(mat_dynamic.cols);
        float mat_dynamic_rows = float(mat_dynamic.rows);
#if 0

        while (true){
#endif

        std::cout << "================parallel_for" << std::endl;








        // 1000 thread for partial cloud to update localization

        tbb::parallel_for(tbb::blocked_range<int>(0, 1000),
                          [&](tbb::blocked_range<int> r) {
                              for (auto it = r.begin(); it != r.end(); it++) {
//                                  std::cout << "================parallel_for " << it << std::endl;

                                  std::valarray<float> cos_val, sin_val;
//                                  float yaw = Partials[it].map_laser_pose.getYaw();

                                  laser_grid.createBeamBase(0.5 * M_PI + Partials[it].map_laser_pose.getYaw(), cos_val,
                                                            sin_val);

                                  ublas::matrix<float> op_beam;
                                  ublas::noalias(op_beam) = ublas::prod(Partials[it].minv_beam, op);
//
                                  float resotion = 0.05;
                                  float match_weight = 1;
//                                  std::vector<float> mask_val(40, 0.0);
                                  float mask_array[40];
                                  float weight_array[sz];


                                  float *op_beam_ptr = op_beam.data().begin();
                                  size_t op_beam_size = op_beam.size2();

                                  /*================================
                                   * conpute weight
                                   *
                                   * */
                                  mp_util::mp_float *mp_weight = &(Partials[it].mp_weight);

                                  for (size_t i = 0; i < sz; i++) {
                                      int s = -laser_ranges[i] / resotion;
                                      s = std::clip(s, -20, 0) + 20;
                                      int e = (max_range - laser_ranges[i]) /
                                              resotion;
                                      e = std::clip(e, 0, 20) + 20;


                                      float pxs[40];
                                      float pys[40];
//                                      std::vector<int> pxsv(40,0.0);
//                                      std::vector<int> pysv(40,0.0);

//                                      std::array<float, 40> pxsvf;
//                                      std::array<float, 40> pysvf;
//                                      std::array<float, 40> mask_val;

                                      std::array<int, 40> pxsvi;
                                      std::array<int, 40> pysvi;


                                      float op_beam_x = op_beam_ptr[i];
                                      float op_beam_y = op_beam_ptr[i + op_beam_size];
                                      float op_beam_cos = cos_val[i];
                                      float op_beam_sin = sin_val[i];


//                                      std::vector<double> x_vec(1000*800*200,4.5);
//                                      auto y_vec = x_vec;

                                      size_t mat_dynamic_cols_1 = mat_dynamic.cols;
                                      size_t mat_dynamic_rows_1 = mat_dynamic.rows;
                                      float laser_resolution_1 = laser_resolution;
                                      unsigned char *input = (unsigned char *) (mat.data);
                                      float *js_ptr = &(js[0]);
//                                      float *xs_ptr = &(pxsvf[0]);
//                                      float *ys_ptr = &(pysvf[0]);
                                      int *xs_ptr_i = &(pxsvi[0]);
                                      int *ys_ptr_i = &(pysvi[0]);
//                                      float *mask_val_ptr = &(mask_val[0]);


//                                      for(int j = 0;j < mat.rows;j++){
//                                          for(int ii = 0;ii < mat.cols;ii++){
//                                              unsigned char b = input[j + ii ] ;
//                                              unsigned char g = input[j + i + 1];
//                                              unsigned char r = input[j + i + 2];
//                                          }
//                                      }

                                      bool track = true;
                                      int f_hit = e - 1;
#pragma omp simd aligned (js_ptr, xs_ptr_i, ys_ptr_i: 32)
                                      for (int j = s; j < e; j++) {


#if 1
                                          xs_ptr_i[j] = std::clip(
                                                  int(round((op_beam_x + js_ptr[j] * op_beam_cos) *
                                                            map_resolution_scale)), int(0),
                                                  int(mat_dynamic_rows_1));
                                          ys_ptr_i[j] = std::clip(
                                                  int(round((op_beam_y + js_ptr[j] * op_beam_sin) *
                                                            map_resolution_scale)), int(0),
                                                  int(mat_dynamic_cols_1));

#endif


                                      }

// todo : test speed
#if 1
//                                      *mp_weight = 1.0;

//                                      #pragma omp parallel for  reduction(+:msum)
                                      for (int j = s; j < e; j++) {
                                          if (j > s) {
                                              if (xs_ptr_i[j] == xs_ptr_i[j - 1] && ys_ptr_i[j] == ys_ptr_i[j - 1])
                                                  continue;
                                          }
                                          // todo test cv mat read speed
                                          float pv = mat.at<uchar>(xs_ptr_i[j], ys_ptr_i[j]);
//                                          float pv = input[xs_ptr_i[j] + ys_ptr_i[j]];
                                          if (pv == 255 || j == e - 1) {

                                              float diff = fabs((j - 20) * laser_resolution);

                                              diff = std::clip(diff, 0.01f, 0.5f);
                                              float normal_weight = glt.data[int(glt.scale * diff)];
                                              // todo check prod
//                                              (*mp_weight) *= normal_weight;
                                              mp_weight->prod_v(normal_weight);
//                                              std::cout << "==== " <<it << std::endl;
//                                              std::cout << "==== " <<it << "\nnormal_weight: " << normal_weight << std::endl;
//                                              mp_weight->print();
//                                              std::cout << "==== " <<it << std::endl;

//                                              exit(33);
//                                              match_weight += diff * diff;
                                              break;

                                          }

                                      }

#endif


                                  }
                                  /*================================
                                   * End conpute weight
                                   */


                                  /* ===============================
                                   * update map
                                   * Q = Q * A * B
                                   *
                                   * 1. Q = Q * A
                                   *
                                   * 2. Q = Q * B
                                   * */

                                  // given occupied points and free points
                                  // create point matrix


                              }

                          },
                          tbb::simple_partitioner());
#if 0
        }
#endif

        timer.stop();
        std::cout << "total time: " << timer.elapsedSeconds() << std::endl;
        auto best_weight = Partials[0].mp_weight;
        size_t best_id = 0;
        for (size_t i = 0; i < 1000; i++) {
//        std::cout << "------\n id " << i << " ---weight: " ;
//            Partials[i].mp_weight.print();
//        std::cout << "pose:\nx: " << Partials[i].map_laser_pose.getX() << " y: "<< Partials[i].map_laser_pose.getY()
//                  << " yaw: " << Partials[i].map_laser_pose.getYaw()<< std::endl;
//
//        if (i == 999){
//            std::cout << "map laser :\n" << Partials[i].map_laser_pose.matrix() << std::endl;
//
//            std::cout << "minv:\n" << Partials[i].minv_beam << std::endl;
//            minv;
//            std::cout << "global minv:\n" << minv << std::endl;
//
//        }

            if (Partials[i].mp_weight > best_weight) {
                best_weight = Partials[i].mp_weight;
                best_id = i;
            }
        }

        std::cout << "best id :" << best_id;
        std::cout << "pose:\nx: " << Partials[best_id].map_laser_pose.getX() << " y: "
                  << Partials[best_id].map_laser_pose.getY()
                  << " yaw: " << Partials[best_id].map_laser_pose.getYaw() << std::endl;

        std::cout << "best weight: " << Partials[best_id].match_weight << std::endl;

        if (Partials[best_id].mp_weight > global_best_weight) {
            global_best_weight = Partials[best_id].mp_weight;
            //global_enable_match = true;
            final_pose.pose.position.x = Partials[best_id].map_laser_pose.getX();
            final_pose.pose.position.y = Partials[best_id].map_laser_pose.getY();

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(Partials[best_id].map_laser_pose.getYaw()),
                                  final_pose.pose.orientation);

        } else {
            // global_enable_match = false;

        }


        final_pose.header.stamp = ros::Time::now();
//        cv::imshow("laser", mat);
//        cv::imshow("laser", mat);

//        cv::imshow("mat_dynamic", mat_dynamic);
        final_pose_pub.publish(final_pose);
        (*initialpose_ptr).pose.pose = final_pose.pose;


        scan2.header.stamp = ros::Time::now();
//        scan_pub0.publish(scan);
//        scan_pub.publish(scan2);
        map_pub.publish(m_map);

        cv::waitKey(1);
    }
    // 4. show in cv mat



}
