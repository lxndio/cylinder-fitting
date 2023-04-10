#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

#include <Ransac.h>
#include <pmp/MatVec.h>
#include <vector>

using namespace pmp;

TEST_CASE("Find connected components (zero components)", "[algorithms]") {
    std::vector<vec3> points;

    Ransac ransac(std::vector<vec3>(), 0.0, 0, 1.0);
    std::vector<std::vector<vec3>> connected_components = ransac.find_connected_components(points);

    REQUIRE(connected_components.size() == 0);
}

TEST_CASE("Find connected components (one component)", "[algorithms]") {
    std::vector<vec3> points;

    for (double x = 0.0; x < 10.0; x += 1.0) {
        for (double y = 0.0; y < 10.0; y += 1.0) {
            for (double z = 0.0; z < 10.0; z += 1.0) {
                points.push_back(vec3(x, y, z));
            }
        }
    }

    Ransac ransac(std::vector<vec3>(), 0.0, 0, 1.0);
    std::vector<std::vector<vec3>> connected_components = ransac.find_connected_components(points);

    REQUIRE(connected_components.size() == 1);
    REQUIRE_THAT(connected_components[0], Catch::Matchers::UnorderedEquals(points));
}

TEST_CASE("Find connected components (two components)", "[algorithms]") {
    std::vector<vec3> points;
    std::vector<vec3> points_in_component_1;
    std::vector<vec3> points_in_component_2;

    for (double x = 0.0; x < 10.0; x += 1.0) {
        for (double y = 0.0; y < 10.0; y += 1.0) {
            for (double z = 0.0; z < 10.0; z += 1.0) {
                vec3 point(x, y, z);
                points.push_back(point);
                points_in_component_1.push_back(point);
            }
        }
    }

    for (double x = 11.5; x < 21.5; x += 1.0) {
        for (double y = 0.0; y < 10.0; y += 1.0) {
            for (double z = 0.0; z < 10.0; z += 1.0) {
                vec3 point(x, y, z);
                points.push_back(point);
                points_in_component_2.push_back(point);
            }
        }
    }

    Ransac ransac(std::vector<vec3>(), 0.0, 0, 1.0);
    std::vector<std::vector<vec3>> connected_components = ransac.find_connected_components(points);

    REQUIRE(connected_components.size() == 2);
    REQUIRE_THAT(connected_components[0], !Catch::Matchers::UnorderedEquals(connected_components[1]));

    REQUIRE_THAT(connected_components[0],
        Catch::Matchers::UnorderedEquals(points_in_component_1) ||
        Catch::Matchers::UnorderedEquals(points_in_component_2)
    );

    REQUIRE_THAT(connected_components[1],
        Catch::Matchers::UnorderedEquals(points_in_component_1) ||
        Catch::Matchers::UnorderedEquals(points_in_component_2)
    );
}

TEST_CASE("Get neighbors of point", "[algorithms]") {
    std::vector<vec3> include;
    std::vector<vec3> exclude = {
        vec3(4.5, 0.0, 0.0),
        vec3(5.5, 0.0, 0.0)
    };

    for (double i = 0.0; i <= 10.0; i += 0.5) {
        include.push_back(vec3(i, 0.0, 0.0));
    }

    vec3 p(5.0, 0.25, 0.0);
    double eps = 1.0;

    SECTION("No include points") {
        std::vector<vec3> include_empty;

        REQUIRE_THAT(
            Ransac::get_neighbors(include_empty, exclude, p, eps),
            Catch::Matchers::Equals(std::vector<vec3>())
        );
    }

    SECTION("No exclude points") {
        std::vector<vec3> exclude_empty;
        std::vector<vec3> result = {
            vec3(4.5, 0.0, 0.0),
            vec3(5.0, 0.0, 0.0),
            vec3(5.5, 0.0, 0.0)
        };

        REQUIRE_THAT(
            Ransac::get_neighbors(include, exclude_empty, p, eps),
            Catch::Matchers::Equals(result)
        );
    }

    SECTION("Include and exclude points") {
        std::vector<vec3> result = {
            vec3(5.0, 0.0, 0.0)
        };

        REQUIRE_THAT(
            Ransac::get_neighbors(include, exclude, p, eps),
            Catch::Matchers::Equals(result)
        );
    }
}

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
