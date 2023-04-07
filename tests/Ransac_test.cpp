#include <gtest/gtest.h>

#include "Ransac.h"
#include "pmp/MatVec.h"

using namespace pmp;

TEST(DistPointLine, CorrectWithPoints) {
    vec3 p(10.0, 10.0, 0.0);
    vec3 l1(5.0, 0.0, 0.0);
    vec3 l2(20.0, 0.0, 0.0);

    EXPECT_EQ(Ransac::dist_point_line(p, l1, l2), 10.0);
}

TEST(DistPointLine, CorrectWithLine) {
    vec3 p(10.0, 10.0, 0.0);
    vec3 l(1.0, 0.0, 0.0);

    EXPECT_EQ(Ransac::dist_point_line(p, l), 10.0);
}
