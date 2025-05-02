
#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
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

template<class hull_t, class points_t>
void save_hull(const hull_t& hull, const points_t& points) {

    size_t nb_dims = hull.dimensions();

    int nb_points = 0;
    std::ofstream point_stream("points.csv");
    for (const auto& point : points) {
        for (int i = 0; i < nb_dims; i++) {
            point_stream << point[i] << ((i == nb_dims - 1) ? '\n' : ',');
        }
        nb_points++;
    }

    std::ofstream face_stream("hull.csv");
    for (const auto& face : hull.faces()) {
        for (int j = 0; j < nb_dims; j++) {
            face_stream << nb_points << ((j == nb_dims - 1) ? '\n' : ',');
            for (int i = 0; i < nb_dims; i++) {
                point_stream << face.points()[j][i] << ((i == nb_dims - 1) ? '\n' : ',');
            }
            nb_points++;
        }

    }
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
template<class T>
T rand_guassian() {
    static std::minstd_rand rand(42);
    return std::normal_distribution<T>(0, 1)(rand);
}

// Generates random points in a box.
template<class T, size_t N, class it>
void rand_points_in_box(boxN<T, N> box, it first, it last) {
    while (first != last) {
        for (size_t i = 0; i < N; i++)
            (*first)[i] = rand(box.min[i], box.max[i]);
        ++first;
    }
}

// Generates random points on (not in!) a box.
template<class T, size_t N, class it>
void rand_points_on_box(boxN<T, N> box, it first, it last) {

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
    rand_points_in_box(box, first, last);
    while (first != last) {
        // Find a random face weighted by face area.
        T area = rand<T>(0, total_area);
        size_t face_index = 0;
        while (area > 0) {
            area -= face_area[face_index++];
        }
        face_index--;
        (*first)[face_index] = (rand(0, 1) ? box.min : box.max)[face_index];
        ++first;
    }
}

// Generates random points on (not in!) a sphere.
template<class T, size_t N, class it>
void rand_points_on_sphere(vecN<T, N> center, T radius, it first, it last) {
    while (first != last) {
        vecN<T, N> point;
        for (size_t i = 0; i < N; i++)
            point[i] = rand_guassian<T>();
        point = center + radius * point.normalized();
        for (size_t i = 0; i < N; i++)
            (*first)[i] = point[i];
        ++first;
    }
}


template<size_t N, class it, class container>
void verify_point_in_container(it point_it, const container& original_points) {

    using T = std::decay_t<decltype((*original_points.begin())[0])>;
    using original_iterator = decltype(original_points.begin());

    original_iterator original_it;
    if constexpr (std::random_access_iterator<original_iterator> && !std::is_void_v<it>) {
        // Get the iterator in the original_points directly.
        auto dist = point_it - original_points.begin();
        if (dist < 0 || dist >= (std::ptrdiff_t)original_points.size())
            make_test_fail("A point in the hull is not in the original container.");

        // Verify that the point iterator and the original_points iterator are compatible.
        original_it = original_points.begin() + dist;
        if (original_it != point_it ||
            original_it < point_it ||
            original_it > point_it ||
            (original_it - point_it) != 0 ||
            (point_it - original_it) != 0)
            make_test_fail("Incompatible point and container iterators.");
    }
    else {
        // Search the container for the original point.
        original_it = std::find_if(original_points.begin(), original_points.end(), [point_it](const auto& point) { return vecN<T, N>(point) == vecN<T, N>(*point_it); });
        if (original_it == original_points.end())
            make_test_fail("A point in the hull is not in the original container.");
    }

    // Verify that the original and hull points are the same.
    auto point = *point_it;
    decltype(point) original_point = *original_it;
    if(point != original_point)
        make_test_fail("The point is not equal to the original point.");
}

// Make sure all the points are from the container and their iterators are compatible.
template<class T, size_t N, class it, class container>
void verify_points_in_container(const convex_hull<T, N, it>& hull, const container& original_points) {

    // Verify all points are points() are from the container.
    for (auto point_it = hull.points().begin(); point_it != hull.points().end(); ++point_it) {
        verify_point_in_container<N>(point_it, original_points);
    }

    // Verify all points from faces().points() are from the container.
    for (const auto& face : hull.faces()) {
        for (auto point_it = face.points().begin(); point_it < face.points().end(); ++point_it) {
            verify_point_in_container<N>(point_it, original_points);
        }
    }
}



// Makes sure that all points from container are inside the hull.
template<class T, size_t N, class it, class container>
void verify_points_in_hull(const convex_hull<T, N, it>& hull, const container& original_points) {

    constexpr T tolerance = (T)(sizeof(T) == 8 ? 1e-4 : 1e-3);

    // Verify all point in container are inside the hull.
    for (const auto& face : hull.faces()) {
        for (const vecN<T, N> point : original_points) {
            auto dist = dist_to_plane(face.plane(), vecN<T, N>(point));
            if (dist > tolerance) {
                make_test_fail("One of the points is outside the hull.");
            }
        }
    }
}

// Makes sure all points are in at least one face.
template<class T, size_t N, class it>
void verify_face_points(const convex_hull<T, N, it>& hull) {

    using point_const_iterator = std::decay_t<decltype(hull.points().begin().base())>;
    std::unordered_map<point_const_iterator, size_t> face_counts;
    for (auto point_it = hull.points().begin(); point_it != hull.points().end(); ++point_it) {
        face_counts[point_it] = 0;
    }

    for (const auto& face : hull.faces()) {
        for (auto point_it = face.points().begin(); point_it != face.points().end(); ++point_it) {
            auto face_count_it = face_counts.find(point_it);
            if (face_count_it == face_counts.end())
                make_test_fail("A face point is not in points().");
            face_count_it->second++;
        }
    }

    for (const auto& [point_it, face_count] : face_counts) {
        if(face_count == 0)
            make_test_fail("A point is not in any face.");
    }
}

// Makes sure the face points are all on the plane.
template<class T, size_t N, class it>
void verify_face_planes(const convex_hull<T, N, it>& hull) {

    constexpr T tolerance = (T)(sizeof(T) == 8 ? 1e-4 : 1e-3);

    for (const auto& face : hull.faces()) {
        for (const vecN<T, N> point : face.points()) {
            auto dist = abs(dist_to_plane(face.plane(), point));
            if (dist > tolerance)
                make_test_fail("Incorrect face plane.");
        }
    }
}

// Makes sure that all faces have an outside winding.
template<class T, class it>
void verify_3d(const convex_hull<T, 3, it>& hull) {

    for (const auto& face : hull.faces()) {
        auto cross = (vec3<T>(face.points()[0]) - vec3<T>(face.points()[1])).cross(vec3<T>(face.points()[0]) - vec3<T>(face.points()[2]));
        auto center_to_face = vec3<T>(face.points()[0]) - hull.center();
        auto dot = cross.dot(center_to_face);
        if(dot < -0.1)
            make_test_fail("Incorrect face winding.");
    }
}


// Makes sure that 2d makes a ccw polygon with the i'th point belonging to the i'th face.
template<class T, class it>
void verify_2d(const convex_hull<T, 2, it>& hull) {

    if (hull.points().size() != hull.faces().size())
        make_test_fail("Incorrect polygon.");

    const auto center = hull.center();
    auto face = hull.faces().begin();
    auto point = hull.points().begin();

    for (size_t i = 0; i < hull.points().size(); i++) {

        // Make sure the neighbors are the prev and next faces.
        auto prev_face = std::prev((face == hull.faces().begin()) ? hull.faces().end() : face);
        if (face->neighbors().begin() != prev_face)
            make_test_fail("Incorrect prev neighbor.");

        auto next_face = std::next(face);
        if (next_face == hull.faces().end())
            next_face = hull.faces().begin();
        if (face->neighbors().begin() + 1 != next_face)
            make_test_fail("Incorrect next neighbor.");

        // Make sure the point is this face's first point.
        auto next_point = std::next(point);
        if (next_point == hull.points().end())
            next_point = hull.points().begin();
        if (face->points().begin() != point || face->points()[0] != *point)
            make_test_fail("Incorrect point.");
        if (face->points().begin() + 1 != next_point || face->points()[1] != *next_point)
            make_test_fail("Incorrect next point.");

        // Make sure the face is ccw.
        vec2<T> center_to_point = vec2<T>(*point) - center;
        vec2<T> point_to_next_point = vec2<T>(*next_point) - vec2<T>(*point);
        auto cross = center_to_point.cross(point_to_next_point);
        if(center_to_point.cross(point_to_next_point) < 0)
            make_test_fail("The hull should be a ccw polygon.");

        point = next_point;
        face = next_face;
    }
}


// Makes sure the convex hull is a single closed mesh with no gaps.
template<class T, size_t N, class it>
void verify_face_neighbors(const convex_hull<T, N, it>& hull) {

    using face_const_iterator = std::decay_t<decltype(hull.faces().begin())>;
    std::unordered_set<face_const_iterator> visited_faces;
    std::unordered_set<face_const_iterator> faces_to_visit;

    // Starting from one face, we should be able to reach all the others.
    faces_to_visit.insert(hull.faces().begin());
    while (!faces_to_visit.empty()) {

        // Pop a face and add it to the visited list.
        auto face = *faces_to_visit.begin();
        faces_to_visit.erase(faces_to_visit.begin());
        visited_faces.insert(face);

        // Add the neighbors to the list if they havent been visited yet.
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
                auto n_points = neighbor->points();
                for (auto neighbor_point = neighbor->points().begin(); neighbor_point != neighbor->points().end(); ++neighbor_point) {
                    if (face_point.base() == neighbor_point.base()) {
                        is_in_common = true;
                        break;
                    }
                }
                bool should_be_in_common = (neighbor - face->neighbors().begin()) != (face_point - face->points().begin());
                if (N == 2)
                    should_be_in_common = !should_be_in_common; // For 2d, the i'th point SHOULD be in the i'th neighbor.
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

template<class T, size_t N, class it, class container>
void verify_hull(const convex_hull<T, N, it>& hull, const container& original_points) {
    verify_face_neighbors(hull);
    verify_face_planes(hull);
    //verify_face_points(hull);
    if constexpr (!std::is_void_v<it>)
        verify_points_in_container(hull, original_points);
    verify_points_in_hull(hull, original_points);
    if constexpr (N == 2)
        verify_2d(hull);
    if constexpr (N == 3)
        verify_3d(hull);
}

template<class it_a, class it_b, bool subtraction, bool equality, bool comparison>
void verify_iterator_comparisons() {
    static_assert(subtraction == requires(it_a a, it_b b) { a - b; b - a; }, "Incorrect iterator subtraction support.");
    static_assert(equality == requires(it_a a, it_b b) { a == b; b == a; }, "Incorrect iterator equality support.");
    static_assert(comparison == requires(it_a a, it_b b) { a <=> b; b <=> a; }, "Incorrect iterator comparison support.");
}

template<class T, size_t N, class it, class container>
void verify_iterators() {

    constexpr bool is_void = std::is_void_v<it>;
    constexpr bool random_access_it = std::random_access_iterator<it>;

    using face_point_it = decltype(std::declval<convex_hull<T, N, it>>().faces().begin()->points().begin());
    using hull_point_it = decltype(std::declval<convex_hull<T, N, it>>().points().begin());
    using container_it = container::iterator;
    using container_const_it = container::const_iterator;

    verify_iterator_comparisons<face_point_it, face_point_it, true, true, true>();
    verify_iterator_comparisons<face_point_it, hull_point_it, random_access_it, true, random_access_it>();
    verify_iterator_comparisons<face_point_it, container_it, random_access_it, !is_void, random_access_it>();
    verify_iterator_comparisons<face_point_it, container_const_it, random_access_it, !is_void, random_access_it>();

    verify_iterator_comparisons<hull_point_it, hull_point_it, false, true, false>();
    verify_iterator_comparisons<hull_point_it, container_it, random_access_it, !is_void, random_access_it>();
    verify_iterator_comparisons<hull_point_it, container_const_it, random_access_it, !is_void, random_access_it>();

}

template<class T, size_t N, class it, class container, class func>
void test_hull_final(func&& create_points) {
    container original_points;
    const size_t nb_original = 100;
    const size_t nb_extra = 100;

    // Reserve enough memory so that we won't have to reallocate and invalid iterators.
    if constexpr (requires { original_points.reserve(0); }) {
        original_points.reserve(nb_original + nb_extra);
    }

    // Test hull creation.
    original_points.resize(nb_original);
    create_points(original_points.begin(), original_points.end());
    convex_hull<T, N, it> hull(original_points.begin(), original_points.end());

    // Test extend() for a single point.
    if constexpr (std::is_void_v<it>) {
        auto& point = original_points.emplace_back();
        create_points(std::prev(original_points.end()), original_points.end());
        hull.extend(point);
        verify_hull(hull, original_points);
    }

    // Test extend() on a range.
    auto first = std::prev(original_points.end());
    original_points.resize(nb_original + nb_extra);
    first = std::next(first);
    create_points(first, original_points.end());
    hull.extend(first, original_points.end());

    verify_hull(hull, original_points);

}

template<class T, size_t N, class it, class container>
void test_hull_choose_shape() {

    verify_iterators<T, N, it, container>();

    boxN<T, N> box;
    for (int i = 1; i <= N; i++) {
        box.min[i - 1] = (T)-i;
        box.max[i - 1] = (T)i;
    }

    test_hull_final<T, N, it, container>([box](auto first, auto last) { rand_points_on_box(box, first, last); });
    test_hull_final<T, N, it, container>([box](auto first, auto last) { rand_points_in_box(box, first, last); });
    test_hull_final<T, N, it, container>([box](auto first, auto last) { rand_points_on_sphere(box.max, (T)1, first, last); });
}

template<class T, size_t N, class container>
void test_hull_choose_iterator() {
    test_hull_choose_shape<T, N, void, container>();
    test_hull_choose_shape<T, N, typename container::iterator, container>();
    test_hull_choose_shape<T, N, typename container::const_iterator, container>();
}

template<class T, size_t N>
void test_hull_choose_container() {
    std::cout << "\nTesting f" << (sizeof(T) * 8) << " in " << N << "d.\n" << colors::yellow << "TESTING..." << colors::white << '\r';
    test_hull_choose_iterator<T, N, std::list<vecN<T, N>>>();
    test_hull_choose_iterator<T, N, std::vector<vecN<T, N>>>();
    test_hull_choose_iterator<T, N, std::vector<std::array<T, N>>>();
    std::cout << colors::green << "PASS                 " << colors::white;
}

template<size_t N>
void test_hull_choose_scalar() {
    test_hull_choose_container<float, N>();
    test_hull_choose_container<double, N>();
}

void test_hull_choose_dimension() {
    test_hull_choose_scalar<2>();
    test_hull_choose_scalar<3>();
    test_hull_choose_scalar<4>();
}

int main() {
    test_hull_choose_dimension();

    std::cout << "\n\nGlobal Result: " << colors::green << "PASS" << colors::white << "\n\n";

    return 0;
}