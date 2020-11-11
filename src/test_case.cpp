#define BOOST_TEST_MODULE eKalmanTestSuit

#include <vector>
#include <iostream>
#include <boost/test/unit_test.hpp>
#include "Eigen/Dense"
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

t_kalmanState 	state;
t_kalmanPredict predict;
t_kalmanState 	predictedState;

BOOST_AUTO_TEST_SUITE(testSuitPredict)
BOOST_AUTO_TEST_CASE(Predict_1)
{
    state.P = MatrixXd(3, 3);
    state.P << 1, 2, 3,
        1, 2, 3,
        1, 2, 3;
    state.x = VectorXd(3);
    state.x << 1, 1, 1;

    predict.F = MatrixXd(3, 3);
    predict.F << 1, 0, 1,
        0, 1, 0,
        0, 0, 1;
    predict.Q = MatrixXd(3, 3);
    predict.Q << 1, 0, 1,
        0, 1, 0,
        0, 0, 1;

    VectorXd truth_x = VectorXd(3);
    truth_x << 2, 1, 1;
    MatrixXd truth_P = MatrixXd(3, 3);
    truth_P << 9, 4, 7,
        4, 3, 3,
        4, 2, 4;

    predictedState = Predict(state, predict);

    BOOST_CHECK_EQUAL(predictedState.x, truth_x);
    BOOST_CHECK_EQUAL(predictedState.P, truth_P);
}

BOOST_AUTO_TEST_CASE(Predict_2)
{
    state.P = MatrixXd(4, 4);
    state.P << 1, 4, 2, 2,
        7, 2, 4, 9,
        9, 7, 5, 2,
        1, 9, 5, 1;
    state.x = VectorXd(4);
    state.x << 1, 8, 6, 7;

    predict.F = MatrixXd(4, 4);
    predict.F << 0, 1, 0, 1,
        1, 0, 0, 1,
        0, 0, 0, 1,
        0, 0, 0, 0;
    predict.Q = MatrixXd(4, 4);
    predict.Q << 6, 7, 6, 9,
        0, 5, 4, 0,
        6, 8, 5, 2,
        8, 5, 5, 7;

    VectorXd truth_x = VectorXd(4);
    truth_x << 15, 8, 7, 0;
    MatrixXd truth_P = MatrixXd(4, 4);
    truth_P << 27, 25, 16, 9,
        16, 10, 7, 0,
        16, 10, 6, 2,
        8, 5, 5, 7;

    predictedState = Predict(state, predict);

    BOOST_CHECK_EQUAL(predictedState.x, truth_x);
    BOOST_CHECK_EQUAL(predictedState.P, truth_P);
}

BOOST_AUTO_TEST_CASE(Predict_3)
{
    state.P = MatrixXd(4, 4);
    state.P << 1, 4, 2, 2,
        7, 2, 4, 9,
        9, 7, 5, 2,
        1, 9, 5, 1;
    state.x = VectorXd(4);
    state.x << 1, 8, 6, 7;

    predict.F = MatrixXd(4, 4);
    predict.F << 1, 1, 1, 0,
        1, 0, 1, 0,
        1, 0, 1, 1,
        0, 0, 1, 0;
    predict.Q = MatrixXd(4, 4);
    predict.Q << 6, 7, 6, 9,
        0, 5, 4, 0,
        6, 8, 5, 2,
        8, 5, 5, 7;

    VectorXd truth_x = VectorXd(4);
    truth_x << 15, 7, 14, 6;
    MatrixXd truth_P = MatrixXd(4, 4);
    truth_P << 47, 35, 47, 20,
        28, 22, 25,  7,
        49, 31, 33, 14,
        29, 19, 21, 12;

    predictedState = Predict(state, predict);

    BOOST_CHECK_EQUAL(predictedState.x, truth_x);
    BOOST_CHECK_EQUAL(predictedState.P, truth_P);
}

BOOST_AUTO_TEST_CASE(Predict_4, *boost::unit_test::tolerance(boost::test_tools::fpc::percent_tolerance(2.0)))
{
    state.P = MatrixXd(5, 5);
    state.P << 1, 4, 2, 2, 7,
        2, 4, 9, 9, 7,
        5, 2, 1, 9, 5,
        1, 6, 7, 6, 9,
        0, 5, 4, 0, 6.;
    state.x = VectorXd(5);
    state.x << 2, 6, 8, 9, 1;

    predict.F = MatrixXd(5, 5);
    predict.F << 0, 1, 0, 1, 0,
        0, 0, 1, 0, 1,
        1, 0, 0, 1, 1,
        1, 0, 1, 1, 0,
        0, 1, 1, 0, 0.;
    predict.Q = MatrixXd(5, 5);
    predict.Q << 8, 5, 2, 8, 5,
        5, 7, 1, 8, 6,
        7, 1, 8, 1, 0,
        8, 8, 8, 4, 2,
        0, 6, 7, 8, 2;

    VectorXd truth_x = VectorXd(5);
    truth_x << 15, 9, 12, 19, 14.;
    MatrixXd truth_P = MatrixXd(5, 5);
    truth_P << 33, 37, 36, 42, 31,
        21, 23, 26, 27, 18,
        30, 36, 40, 24, 28,
        37, 39, 53, 38, 24,
        24, 28, 44, 43, 18;

    predictedState = Predict(state, predict);
    
    BOOST_CHECK_EQUAL(predictedState.x, truth_x);
    BOOST_CHECK_EQUAL(predictedState.P, truth_P);
}
BOOST_AUTO_TEST_SUITE_END()