// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values
// are almost equal (4
// ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values
// are almost equal (4
// ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between
// val1 and val2
// doesn't exceed the given absolute error
//=======================================================================================================================================================

#include <ros/ros.h>
#include "gtest/gtest.h"

// This module test should only test the ros functionality of any node (does it run, does it respond
// correctly to
// messages).
// The actual functionality should be tested in the library that implements it.
// Any test acts like a node and can receive and send messages to your nodes/nodelets to test their
// behaviour
// You can even control time by setting "use_sim_time" in the .test-file and advertising the /clock
// topic.
// Keep in mind that:
// - your node needs some time to initialize and subscribe to your topics
// - if you control /clock, nothing might happen until you send the first clock message
// - it takes time until a node responds to your messages
// - utils_testing_ros provides functionality to make your testing life easier


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "mono_lidar_test");
    // The async spinner lets you publish and receive messages during the tests, no need to call
    // spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
