// #include <gtest/gtest.h>

// // using namespace pmp;

// TEST(DistPointLine, CorrectWithPoints) {
//     // vec3 p(10.0, 10.0, 0.0);
//     // vec3 l1(5.0, 0.0, 0.0);
//     // vec3 l2(20.0, 0.0, 0.0);
//     EXPECT_EQ(1, 1);

//     //EXPECT_EQ(Ransac::dist_point_line(p, l1, l2), 10.0);
// }

#include <catch2/catch_test_macros.hpp>

#include <Ransac.h>
#include <pmp/MatVec.h>

using namespace pmp;

TEST_CASE("Compute distance of point to line (defined by two points)", "[math]") {
    vec3 p(10.0, 10.0, 0.0);
    vec3 l1(5.0, 0.0, 0.0);
    vec3 l2(20.0, 0.0, 0.0);
    
    REQUIRE(Ransac::dist_point_line(p, l1, l2) == 10.0);
}

TEST_CASE("Compute distance of point to line (defined by point and direction)", "[math]") {
    vec3 p(10.0, 10.0, 0.0);
    vec3 l1(5.0, 0.0, 0.0);
    vec3 d(1.0, 0.0, 0.0);

    REQUIRE(Ransac::dist_point_line_direction(p, l1, d) == 10.0);
}
