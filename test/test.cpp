
#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <unordered_set>
#include <bit>

#include "../header/quickhull.h"

using namespace palla;

// Console color codes.
namespace colors {
    static const char* const white = "\033[0m";
    static const char* const green = "\033[92m";
    static const char* const yellow = "\033[93m";
    static const char* const red = "\033[91m";
}

// Utility function to terminate the test.
void make_test_fail(const char* text) {
    std::cout << colors::red << "\nFAIL: " << colors::white << text << "\n\n";
    std::exit(0);
}

// Specialize std::hash for iterators.
namespace std {
    template<class T>
        requires(sizeof(T) == sizeof(size_t))
    class hash<T> {
    public:
        size_t operator()(const T& val) const {
            return std::bit_cast<size_t>(val);
        }
    };
}

// Generates random number of type T in [start, end].
template<class T>
T rand(T start = 0, T end = 1) {
    static std::minstd_rand rand(42);
    if constexpr (std::is_floating_point_v<T>)
        return std::uniform_real_distribution<T>(start, end)(rand);
    else // For some stupid reason uniform_int_distribution does not accept u8!?!?
        return (T)std::uniform_int_distribution<std::conditional_t<sizeof(T) == 1, std::int16_t, T>>(start, end)(rand);
}

// Generates random points in a box.
template<class T, size_t N>
std::vector<vecN<T, N>> rand_points_in_box(boxN<T, N> box, size_t nb_points) {
    std::vector<vecN<T, N>> points(nb_points);
    for (size_t point_index = 0; point_index < points.size(); point_index++) {
        for (size_t i = 0; i < N; i++)
            points[point_index][i] = rand(box.min[i], box.max[i]);
    }
    return points;
}

// Generates random points on (not in!) a box.
template<class T, size_t N>
std::vector<vecN<T, N>> rand_points_on_box(boxN<T, N> box, size_t nb_points) {

    // Find the "area" of each face (there are 2N faces but opposite faces have the same area).
    std::array<T, N> face_area;
    T total_area = 0;
    for (size_t i = 0; i < N; i++) {
        face_area[i] = 1;
        for (size_t j = 0; j < N; j++) {
            if (i != j)
                face_area[i] *= box.size()[j];
        }
        total_area += face_area[i];
    }

    // Create random points in the box and assign them to a random face.
    auto points = rand_points_in_box(box, nb_points);
    for (auto& point : points) {
        // Find a random face weighted by face area.
        T area = rand<T>(0, total_area);
        size_t face_index = 0;
        while (area > 0) {
            area -= face_area[face_index++];
        }
        face_index--;
        point[face_index] = (rand(0, 1) ? box.min : box.max)[face_index];
    }

    return points;
}

// Generates random points on (not in!) a sphere.
template<class T, size_t N>
std::vector<vecN<T, N>> rand_points_on_sphere(vecN<T, N> center, T radius, size_t nb_points) {
    static std::minstd_rand rand(42);
    std::normal_distribution<T> guassian(0, 1);

    std::vector<vecN<T, N>> points(nb_points);
    for (size_t point_index = 0; point_index < points.size(); point_index++) {
        auto& point = points[point_index];
        for (size_t i = 0; i < N; i++)
            point[i] = guassian(rand);
        point = center + radius * point.normalized();
    }

    return points;
}

template<class T, size_t N>
void test_box() {

    boxN<T, N> box;
    box.min.fill((T)-1);
    box.max.fill((T)1);
    auto all_points = rand_points_in_box(box, 1000);

    // Set the actual corners of the box.
    size_t nb_corners = (size_t)1 << N;
    for (size_t corner_index = 0; corner_index < nb_corners; corner_index++) {
        for (size_t i = 0; i < N; i++) {
            all_points[corner_index][i] = (T)1.1 * ((corner_index & ((size_t)1 << i)) ? box.max[i] : box.min[i]);
        }
    }

    convex_hull hull(all_points.begin(), all_points.end());

    if (hull.dimensions() != N)
        make_test_fail("Wrong dimension.");

    std::cout << N << ": " << hull.points().size() << " points and " << hull.faces().size() << " faces.\n";

    auto points = hull.points();
    auto faces = hull.faces();

    std::vector<size_t> point_face_counts;
    for (auto it = points.begin(); it != points.end(); ++it) {
        size_t face_count = 0;
        for (const auto& face : faces) {
            auto face_points = face.points();
            for (auto it2 = face_points.begin(); it2 != face_points.end(); ++it2) {
                face_count += it == it2;
            }
        }
        point_face_counts.push_back(face_count);
    }

    printf("");

}


// Makes sure the convex hull is a single closed mesh with no gaps.
template<class T, size_t N, class it>
void verify_neighbors(const convex_hull<T, N, it>& hull) {

    using face_const_iterator = std::decay_t<decltype(hull.faces().begin())>;
    using point_const_iterator = std::decay_t<decltype(hull.faces().begin()->points().begin().base())>;
    std::unordered_set<face_const_iterator> visited_faces;
    std::unordered_set<face_const_iterator> faces_to_visit;

    // Starting from one face, we should be able to reach all the others.
    faces_to_visit.insert(hull.faces().begin());
    while (!faces_to_visit.empty()) {

        // Pop a face and add it to the visited list.
        auto face = *faces_to_visit.begin();
        faces_to_visit.erase(faces_to_visit.begin());
        visited_faces.insert(face);

        // Add the neighbors on the list if it hasnt been visited yet.
        for (auto neighbor = face->neighbors().begin(); neighbor != face->neighbors().end(); ++neighbor) {
            if (!visited_faces.contains(neighbor))
                faces_to_visit.insert(neighbor);
        }
    }

    // At this point the entire hull should have been visited.
    if(visited_faces.size() != hull.faces().size())
        make_test_fail("The hull is disjoint.");

    // Check each face to make sure there are no holes.
    for (auto face = hull.faces().begin(); face != hull.faces().end(); ++face) {
        for (auto neighbor = face->neighbors().begin(); neighbor != face->neighbors().end(); ++neighbor) {
            // Check that the neighbor exists.
            if (neighbor == face_const_iterator{})
                make_test_fail("The hull is not water-tight.");

            // All the neighbor's points should be in this face except the i'th one.
            for (auto face_point = face->points().begin(); face_point != face->points().end(); ++face_point) {
                bool is_in_common = false;
                for (auto neighbor_point = neighbor->points().begin(); neighbor_point != neighbor->points().end(); ++neighbor_point) {
                    if (face_point.base() == neighbor_point.base()) {
                        is_in_common = true;
                        break;
                    }
                }
                bool should_be_in_common = (neighbor - face->neighbors().begin()) != (face_point - face->points().begin());
                if (N == 2)
                    should_be_in_common = !should_be_in_common;
                if(is_in_common != should_be_in_common)
                    make_test_fail("The neighbor doesn't contain the right points.");
            }

            // Check that the neighbor has this face as its neighbor too.
            bool found_face = false;
            for (auto neighbor_neighbor = neighbor->neighbors().begin(); neighbor_neighbor != neighbor->neighbors().end(); ++neighbor_neighbor) {
                if (neighbor_neighbor == face) {
                    found_face = true;
                    break;
                }
            }
            if(!found_face)
                make_test_fail("Neighborness should be 2-way.");
        }
    }
}



int main() {



    constexpr int nb_points = 1000;
    constexpr int nb_points2 = 0;
    constexpr int nb_dims = 3;

    boxN<float, nb_dims> box;
    box.min.fill(-1);
    box.max.fill(1);
    std::vector<vecN<float, nb_dims>> points;
    points = rand_points_in_box(box, nb_points);
    points.reserve(nb_points + nb_points2);

    points.erase(std::remove_if(points.begin(), points.end(), [](const auto& point) { return point.sqr_norm() > 1; }), points.end());

    using it = decltype(points.begin());
    printf("before: ");
    auto hull = palla::convex_hull<float, nb_dims, void>(points.begin(), points.end());

    std::vector<vecN<float, nb_dims>> hull_points(hull.points().size());
    auto point_it = hull.points().begin();
    for (size_t i = 0; i < hull_points.size(); i++) {
        hull_points[i] = *(point_it++);
    }

    auto face = hull.faces().end();
    face--;


    //point
    //for (const auto& point : hull.faces().begin()->points()) {
    //    point[0];
    //}

    printf("%zi\n", hull.faces().size());

    //verify_neighbors(hull);

    printf("after: ");


    box.min.fill(2);
    box.max.fill(3);
    auto new_points = rand_points_in_box(box, nb_points2);
    points.insert(points.end(), new_points.begin(), new_points.end());

    printf(" ");

    hull.extend(points.end() - nb_points2, points.end());

    printf("%zi\n", hull.faces().size());
    printf("\n");

    {
        std::ofstream out("points.csv");
        for (const auto& point : points) {
            for (int i = 0; i < nb_dims; i++) {
                out << point[i] << ((i == nb_dims - 1) ? '\n' : ',');
            }
        }
        out.close();
    }


    {
        std::ofstream out("hull.csv");
        for (const auto& face : hull.faces()) {
            auto face_points = face.points();
            for (int i = 0; i < nb_dims; i++) {
                out << &face_points[i] - points.data() << ((i == nb_dims - 1) ? '\n' : ',');
            }
        }
        out.close();
    }
}