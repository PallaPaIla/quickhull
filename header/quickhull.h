#pragma once

#include <ranges>

#include "geometry.h"
#include "nway_partition.h"
#include "vec_list.h"

#if __has_include("thread_pool.h")
#include "thread_pool.h"
#else

// Create a dummy thread pool rather than changing everything.
namespace palla {
    namespace details {
        namespace quickhull_namespace {
            class thread_pool {
            public:
                // Public types.

                // A dummy sub pool which holds no threads.
                class sub_pool {
                    friend class thread_pool;
                private:
                    size_t m_desired_size = 0;
                    sub_pool(size_t desired_size) : m_desired_size(desired_size) {}
                public:
                    // sub_pool is default constructible and movable, but not copyable.
                    sub_pool() = default;

                    void release() { m_desired_size = 0; }
                    size_t size() const { return 0; }
                    size_t desired_size() const { return m_desired_size; }
                    bool empty() const { return size() == 0; }
                    bool full() const { return size() == desired_size(); }

                    // Dispatchers.
                    template<class F, class R = std::decay_t<decltype(std::declval<F>()(0))>>
                    auto dispatch_to_reserved(F&& func) {
                        if constexpr (!std::is_void_v<R>)
                            return std::vector<R> {};
                    }
                    template<class F, class R = std::decay_t<decltype(std::declval<F>()(0))>>
                    auto dispatch_to_at_least_one(F&& func) {
                        if constexpr (std::is_void_v<R>)
                            func(0);
                        else
                            return std::vector<R> { func(0) };
                    }
                    template<class F, class R = std::decay_t<decltype(std::declval<F>()(0))>>
                    auto dispatch_to_all(F&& func) {
                        if constexpr (std::is_void_v<R>) {
                            for (int i = 0; i < (int)m_desired_size; i++) {
                                func(i);
                            }
                        }
                        else {
                            std::vector<R> results(m_desired_size);
                            for (int i = 0; i < (int)m_desired_size; i++) {
                                results[i] = func(i);
                            }
                            return results;
                        }
                    }
                };

                // Public functions.
                static thread_pool get() { return {}; }
                void disable() {}
                void enable() {}
                void resize(size_t) {}
                size_t size() const { return 0; }
                bool empty() const { return true; }
                size_t nb_available() const { return 0; }
                size_t nb_working() const { return 0; }
                bool is_worker() const { return false; }
                sub_pool reserve(size_t nb_desired) { return sub_pool(nb_desired); }
            };
        }
    }
}
#endif

namespace palla {
    namespace details {
        namespace quickhull_namespace {


            // Wraps an iterator if it exists. Used as a base class so EBO can take over.
            // We need to abstract the iterator behind a method because trying to access m_it directly gives errors if it doesn't exist, even inside if constexpr.
            template<class it>
            struct iterator_wrapper {
                it m_it;
                auto iterator() const { return m_it; }
            };

            template<>
            struct iterator_wrapper<void> {
                void iterator() const {}
            };

            // Another EBO trick to enable or disable m_face_count.
            template<bool>
            struct conditional_face_count { std::uint32_t m_face_count = 0; };

            template<>
            struct conditional_face_count<false> {};



            // An iterator that increments/decrements like movement_it, but dereferences like dereference_it.
            // Say we have a std::vector<std::vector<int>> and we want to make an iterator that passes through the top vector but dereferences directly to int.
            // movement_it = std::vector<std::vector<int>>::iterator 
            // dereference_it = std::vector<int>::iterator
            // converter = [](const std::vector<std::vector<int>>::iterator& it) { return *it; }
            template<class movement_it, auto converter>
            class transformed_iterator {
            private:
                movement_it m_it;

            public:
                using dereference_it = std::decay_t<decltype(converter(std::declval<movement_it>()))>;
                using difference_type = movement_it::difference_type;
                using value_type = dereference_it::value_type;

                // Constructor.
                transformed_iterator() = default;
                transformed_iterator(movement_it it) : m_it(it) {}

                // Conversion.
                dereference_it base() const { return converter(m_it); }
                operator dereference_it() const { return base(); }

                // Dereferencing.
                auto& operator*() const { return *static_cast<dereference_it>(*this); }
                auto* operator->() const { return &**this; }

                // Enable various movements depending on the movement iterator.
                transformed_iterator& operator++() requires requires { ++m_it; } { ++m_it; return *this; }
                transformed_iterator operator++(int) requires requires { m_it++; } { auto other = *this; ++m_it; return other; }
                transformed_iterator& operator--() requires requires { --m_it; } { --m_it; return *this; }
                transformed_iterator operator--(int) requires requires { m_it--; } { auto other = *this; --m_it; return other; }
                transformed_iterator& operator+=(difference_type i) requires requires { m_it += i; } { m_it += i; return *this; }
                transformed_iterator& operator-=(difference_type i) requires requires { m_it -= i; } { m_it -= i; return *this; }
                transformed_iterator operator+(difference_type i) const requires requires { m_it + i; } { return transformed_iterator(m_it + i); }
                transformed_iterator operator-(difference_type i) const requires requires { m_it - i; } { return transformed_iterator(m_it - i); }
                friend transformed_iterator operator+(difference_type i, const transformed_iterator& it) requires requires { i + it.m_it; } { return *this + i; }

                // Enable operator[] if the movement iterator is random access.
                auto& operator[](difference_type i) const requires requires { m_it[i]; } { return *(transformed_iterator)(m_it + i); }

                // Enable operator- if the movement iterator is random access.
                auto operator-(const transformed_iterator other) const requires requires { m_it - other.m_it; } { return m_it - other.m_it; }

                // Enable operator- with the dereferenced iterator.
                friend auto operator-(const transformed_iterator a, const dereference_it& b) requires requires { b - b; } { return a.base() - b; }
                friend auto operator-(const dereference_it a, const transformed_iterator& b) requires requires { a - a; } { return a - b.base(); }

                // Enable operator- with other transformed iterators if the dereferenced iterators are random access.
                template<class other_movement_it, auto other_converter>
                requires requires(dereference_it a, typename transformed_iterator<other_movement_it, other_converter>::dereference_it b) { a - b; }
                friend auto operator-(const transformed_iterator& a, const transformed_iterator<other_movement_it, other_converter>& b) { return a.base() - b.base(); }

                // Enable comparisons.
                bool operator==(const transformed_iterator&) const requires std::equality_comparable<movement_it> = default;
                auto operator<=>(const transformed_iterator&) const requires std::three_way_comparable<movement_it> = default;

                // Enable comparisons with other transformed iterators when the dereferenced iterators are comparable.
                template<class other_movement_it, auto other_converter>
                requires std::equality_comparable_with<dereference_it, typename transformed_iterator<other_movement_it, other_converter>::dereference_it>
                friend bool operator==(const transformed_iterator& a, const transformed_iterator<other_movement_it, other_converter>& b) { return a.base() == b.base(); }
                template<class other_movement_it, auto other_converter>
                requires std::three_way_comparable_with<dereference_it, typename transformed_iterator<other_movement_it, other_converter>::dereference_it>
                friend auto operator<=>(const transformed_iterator& a, const transformed_iterator<other_movement_it, other_converter>& b) { return a.base() <=> b.base(); }
            };



            // A wrapper around a container. Used when the container knows its size even though the iterators dont.
            template<class container, auto converter = 0>
            class sized_view: public std::ranges::view_interface<sized_view<container, converter>> {

            private:
                const container* m_container = nullptr;
                using base_iterator = std::decay_t<decltype(m_container->cbegin())>;
                using it = std::conditional_t<std::is_same_v<std::decay_t<decltype(converter)>, int>, base_iterator, transformed_iterator<base_iterator, converter>>;

            public:
                sized_view() = default;
                sized_view(const container& container) : m_container(&container) {}

                it begin() const { return m_container->begin(); }
                it end() const { return m_container->end(); }
                size_t size() const { return m_container->size(); }
            };



            // An implementation of quickhull that can be extended several times.
            template<class T, size_t N, class it = void>
            class convex_hull {

                // Various sanity checks.
                static_assert(N >= 2, "A convex hull needs at least 2 dimensions.");
                static_assert(std::is_floating_point_v<T>, "T should be a floating point type, even if the underlying data are integers.");
                static_assert(std::is_void_v<it> || std::bidirectional_iterator<it>, "The iterator type should either be a bidirectional iterator or void.");

            private:

                // Private types.

                // The data for points. The members m_it and m_face_count can be EBO'ed away.
                struct point_wrapper : public iterator_wrapper<it>, public conditional_face_count<(N > 2)> {
                    vecN<T, N> m_point;                 // A minimal copy of the point coordinates so we don't have to dereference and cast to T all the time.
                    std::uint32_t m_unique_index = 0;   // A unique index used for sorting.

                    // Accesses either the copy or the underlying iterator.
                    auto iterator() const {
                        if constexpr (std::is_void_v<it>)
                            return &m_point;
                        else
                            return iterator_wrapper<it>::iterator();
                    }

                };


                // Forward declare face so we can create the iterators.
                class face;


                // Mutable iterators are private.
                using face_iterator = vec_list<face>::iterator;
                using face_const_iterator = vec_list<face>::const_iterator;
                using point_iterator = vec_list<point_wrapper>::iterator;
                using point_const_iterator = vec_list<point_wrapper>::const_iterator;

            public:

                // Public types.

                // A face in the convex hull.
                class face {
                    friend class convex_hull;

                private:
                    size_t m_unique_index = 0;                                                          // A unique index used for sorting.
                    planeN<T, N> m_plane;                                                               // The face's plane, pointing outwards.
                    std::conditional_t<N == 2, point_wrapper, std::array<point_iterator, N>> m_points;  // Iterators to points. For 2d, store 1 point directly and the other is m_neighbors[1]'s.
                    std::array<face_iterator, N> m_neighbors;                                           // Iterators to neighboring faces.

                public:
                    auto plane() const { return m_plane; }

                    auto points() const {
                        if constexpr (N > 2) {
                            using container = std::decay_t<decltype(m_points)>;
                            using base_iterator = std::decay_t<decltype(m_points.begin())>;
                            return sized_view < container, [](base_iterator point) { return (*point)->iterator(); } > (m_points);
                        }
                        else {
                            class point_view : public std::ranges::view_interface<point_view> {
                            private:
                                friend face;
                                point_view(const point_wrapper* a, const point_wrapper* b) : m_points{ a, b } {}

                                std::array<const point_wrapper*, 2> m_points;

                                using base_iterator = std::decay_t<decltype(m_points.cbegin())>;
                                using self_iterator = transformed_iterator < base_iterator, [](base_iterator point) { return (*point)->iterator(); } > ;
                            public:
                                point_view() = default;
                                size_t size() const { return 2; }
                                self_iterator begin() const { return m_points.begin(); }
                                self_iterator end() const { return m_points.end(); }
                            };
                            return point_view(&m_points, &m_neighbors[1]->m_points);
                        }
                    }

                    auto neighbors() const {
                        using container = std::decay_t<decltype(m_neighbors)>;
                        using base_iterator = std::decay_t<decltype(m_neighbors.begin())>;
                        return sized_view<container, [](base_iterator face) -> face_const_iterator { return *face; }>(m_neighbors);
                    }

                };



            private:

                // Private members.
                vec_list<face> m_faces;                         // The faces of the hull.
                vec_list<point_wrapper> m_points;               // The points of the hull.
                std::array<point_wrapper, N * 2> m_extremes;    // The extreme points that form the bounding box.
                size_t m_dimensions = 0;                        // The dimensionality of the points. If it is smaller than N, the convex hull will not be created.
                std::uint32_t m_next_face_index = 0;            // Used as a sort of uuid for faces.
                std::uint32_t m_next_point_index = 0;           // Used as a sort of uuid for point.


                // Private functions.

                // Face/point management. None of these are used for N=2.

                // Add a new point. It will be automatically erased once all its faces are erased.
                point_iterator insert_point(point_wrapper point) { return insert_point(point, m_points.end()); }
                point_iterator insert_point(point_wrapper point, point_iterator pos);

                // Add a face before pos and pointing away from center. If pos is not specified, it will be added at the end.
                face_iterator insert_face(std::span<const point_iterator> points, vecN<T, N> center) { return insert_face(points, center, m_faces.end()); }
                face_iterator insert_face(std::span<const point_iterator> points, vecN<T, N> center, face_iterator pos);

                // Removes a face.
                void erase_face(face_iterator pos);


                // Initialization.

                // Initializes a new hull.
                void initialize(std::span<point_wrapper> points);

                // Initializes to a simplex and returns the center of the simplex.
                [[nodiscard]] vecN<T, N> initialize_to_simplex(std::span<const point_wrapper> points);

                // Optimizations for 2d initialization.
                [[nodiscard]] size_t optim_2d_initialize(std::span<point_wrapper> points) requires (N == 2);
                static void optim_2d_recursion_multithread(vec_list<face>& polygon,
                    std::array<std::span<point_wrapper>, 2> points,
                    std::array<point_wrapper, 2> new_vertices,
                    std::array<face_iterator, 2> next_vertices,
                    T epsilon) requires (N == 2);
                static void optim_2d_recursion(vec_list<face>& polygon, std::span<point_wrapper> points, face_iterator vertex_b, T epsilon) requires (N == 2);


                // Actual extend function.
                void extend_impl(std::span<point_wrapper> points, vecN<T, N> center, T epsilon);


                // The tolerance for extending faces. If a point is closer to a face than this, it will be counted as in the face.
                T epsilon() const { return box().diagonal() * (T)1e-6; }


                // Various static helpers.
                static std::array<point_wrapper, 2 * N> merge_extremes(std::span<const point_wrapper> a, std::span<const point_wrapper> b);
                static std::array<point_wrapper, 2 * N> find_extremes(std::span<const point_wrapper> points);
                static std::vector<point_wrapper>       find_simplex_points(std::span<const point_wrapper> extremes, std::span<const point_wrapper> points, T epsilon);
                static auto                             find_farthest_point_to_each_face(std::span<const face_iterator> faces, std::span<point_wrapper> points, T epsilon);

                template<class it2>
                static std::vector<point_wrapper>       convert_to_point_wrapper(it2 first, it2 last);

            public:

                // Public functions.

                // Constructors.
                convex_hull() = default;

                // Convex hull is movable but not copyable since it holds iterators to itself which would need updating.
                convex_hull(const convex_hull&) = delete;
                convex_hull& operator=(const convex_hull&) = delete;
                convex_hull(convex_hull&&) = default;
                convex_hull& operator=(convex_hull&&) = default;

                template<class it2>
                    requires (std::is_void_v<it> || std::is_same_v<it, it2>)
                convex_hull(it2 first, it2 last) { extend(first, last); }


                // Reset the convex hull.
                template<class it2>
                    requires (std::is_void_v<it> || std::is_same_v<it, it2>)
                void assign(it2 first, it2 last) { clear(); extend(first, last); }
                void clear() {
                    m_faces.clear();
                    m_points.clear();
                    m_extremes = {};
                    m_dimensions = 0;
                }


                // Extend the convex hull to new points.
                template<class it2>
                    requires (std::is_void_v<it> || std::is_same_v<it, it2>)
                void extend(it2 first, it2 last);

                // Extend the hull to a single new point.
                // Only available if "it" is void. Otherwise the point will have no iterator.
                void extend(vecN<T, N> point) requires std::is_void_v<it>;


                // The actual dimensions of the data passed to the convex hull, regardless of N.
                // If this is not equal to N, the convex hull will be empty and a lower dimensional convex hull might be appropriate.
                size_t dimensions() const { return m_dimensions; }
                bool empty() const { return m_faces.empty(); }


                // Bounding box.
                boxN<T, N> box() const;
                vecN<T, N> center() const { return box().center(); }


                // Accessors.
                auto faces() const {
                    return sized_view<vec_list<face>>(m_faces);
                }

                auto points() const {
                    if constexpr (N > 2) {
                        return sized_view<vec_list<point_wrapper>, [](point_const_iterator point) { return point->iterator(); }>(m_points);
                    }
                    else {
                        return sized_view<vec_list<face>, [](face_const_iterator face) { return face->m_points.iterator(); }>(m_faces);
                    }
                }

            };


            // Helper function to deduce the arguments.
            template<size_t N, class cont>
            auto make_convex_hull(cont&& container) {
                using it = std::conditional_t<std::is_reference_v<cont>, std::decay_t<decltype(container.begin())>, void>;
                using T = std::decay_t<decltype((*container.begin())[0])>;
                return convex_hull<T, N, it>(container.begin(), container.end());
            }


            template<class T, size_t N, class it>
            convex_hull<T, N, it>::point_iterator convex_hull<T, N, it>::insert_point(point_wrapper point, point_iterator pos) {

                static_assert(N > 2, "This function should never be called with in 2 dimensions.");

                auto point_iterator = m_points.insert(pos, point);
                point_iterator->m_unique_index = m_next_point_index++;
                return point_iterator;
            }

            template<class T, size_t N, class it>
            convex_hull<T, N, it>::face_iterator convex_hull<T, N, it>::insert_face(std::span<const point_iterator> points, vecN<T, N> center, face_iterator pos) {

                static_assert(N > 2, "This function should never be called with in 2 dimensions.");

                // Create a face with a unique index.
                auto face = m_faces.emplace(pos);
                face->m_unique_index = m_next_face_index++;

                // Add the points to the face and increment their reference count.
                std::array<vecN<T, N>, N> points_for_plane;
                for (size_t i = 0; i < N; i++) {
                    points[i]->m_face_count++;
                    face->m_points[i] = points[i];
                    points_for_plane[i] = points[i]->m_point;
                }

                // Compute the face's plane.
                std::optional<planeN<T, N>> plane_opt = plane_from_points<T, N>(points_for_plane);
                assert(plane_opt); // Cannot make a plane from these points!
                face->m_plane = *plane_opt;
                if (dist_to_plane(face->m_plane, center) > 0)
                    face->m_plane = face->m_plane.flip();

                return face;
            }


            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::erase_face(face_iterator face) {

                static_assert(N > 2, "This function should never be called with in 2 dimensions.");

                // Decrement the reference count for all points in the face. If one of them goes to 0, delete it.
                for (auto point : face->m_points) {
                    if (--point->m_face_count == 0)
                        m_points.erase(point);
                }
                m_faces.erase(face);
            }


            // Creates a box from a list of extreme points.
            template<class T, size_t N, class it>
            boxN<T, N> convex_hull<T, N, it>::box() const {
                boxN<T, N> box;
                for (size_t i = 0; i < N; i++) {
                    box.min[i] = m_extremes[i].m_point[i];
                    box.max[i] = m_extremes[i + N].m_point[i];
                }
                return box;
            }



            // Given two sets of extreme points, merge them.
            template<class T, size_t N, class it>
            std::array<typename convex_hull<T, N, it>::point_wrapper, 2 * N> convex_hull<T, N, it>::merge_extremes(std::span<const point_wrapper> a, std::span<const point_wrapper> b) {
                assert(a.size() == 2 * N && b.size() == 2 * N); // Inconsistent extremes size!

                std::array<point_wrapper, 2 * N> merge;
                for (size_t i = 0; i < N; i++) {
                    merge[i] = (a[i].m_point[i] < b[i].m_point[i]) ? a[i] : b[i];
                    merge[i + N] = (a[i + N].m_point[i] > b[i + N].m_point[i]) ? a[i + N] : b[i + N];
                }
                return merge;
            }



            // Finds the points that touch the axis aligned bounding box.
            // The points are returned as [minX, minY, minZ... maxX, maxY, maxZ...].
            // Uses multiple threads if possible.
            template<class T, size_t N, class it>
            std::array<typename convex_hull<T, N, it>::point_wrapper, 2 * N> convex_hull<T, N, it>::find_extremes(std::span<const point_wrapper> points) {
                assert(!points.empty());    // This function doesn't handle 0 points.

                constexpr size_t CHUNK_SIZE = 1024;

                // Dispatch to several threads.
                auto thread_results = thread_pool::get().reserve((size_t)std::ceil((T)points.size() / CHUNK_SIZE)).dispatch_to_all([points](size_t chunk_index) {

                    auto start = chunk_index * CHUNK_SIZE;
                    auto end = std::min(points.size(), (chunk_index + 1) * CHUNK_SIZE);

                    std::array<T, N * 2> extreme_values;
                    std::array<size_t, N * 2> extreme_indices;

                    for (size_t i = 0; i < N; i++) {
                        extreme_values[i] = infinity;
                        extreme_values[i + N] = -infinity;
                        extreme_indices[i] = 0;
                        extreme_indices[i + N] = 0;
                    }

                    for (auto index = start; index != end; index++) {
                        const auto& point = points[index];
                        for (size_t i = 0; i < N; i++) {
                            T val = point.m_point[i];
                            if (val < extreme_values[i]) {
                                extreme_values[i] = val;
                                extreme_indices[i] = index;
                            }
                            if (val > extreme_values[i + N]) {
                                extreme_values[i + N] = val;
                                extreme_indices[i + N] = index;
                            }
                        }
                    }

                    std::array<point_wrapper, N * 2> extreme_points;
                    for (size_t i = 0; i < N; i++) {
                        extreme_points[i] = points[extreme_indices[i]];
                        extreme_points[i + N] = points[extreme_indices[i + N]];
                    }

                    return extreme_points;
                });

                // Aggregate the results from all threads.
                for (size_t i = 1; i < thread_results.size(); i++) {
                    thread_results[0] = merge_extremes(thread_results[0], thread_results[i]);
                }
                return thread_results[0];
            }



            // Finds the (unique) farthest points to each face. If 2 faces have the same farthest point, it will only be counted once.
            // Also partitions the points so that the ones outside come before the ones inside.
            // The farthest points are also removed, despite being outside.
            template<class T, size_t N, class it>
            auto convex_hull<T, N, it>::find_farthest_point_to_each_face(std::span<const face_iterator> faces, std::span<point_wrapper> points, T epsilon) {

                using bit_type = std::uint64_t;
                constexpr size_t NB_BITS = sizeof(bit_type) * 8;

                struct face_it_and_farthest_index {
                    face_iterator face_it;
                    T dist = 0;
                    size_t farthest_index = 0;
                };

                struct face_it_and_farthest_point {
                    face_iterator face_it;
                    point_wrapper farthest_point;
                };

                // Dispatch to several threads.
                std::atomic<size_t> shared_index{};
                constexpr size_t CHUNK_SIZE = 1024;
                size_t nb_desired_threads = (size_t)std::ceil((T)faces.size() * points.size() / CHUNK_SIZE);
                auto thread_results = thread_pool::get().reserve(nb_desired_threads).dispatch_to_at_least_one([faces, points, epsilon, &shared_index](size_t) {

                    struct {
                        std::vector<bit_type> points_to_keep;
                        std::vector<face_it_and_farthest_index> farthest_indices;
                    } thread_result;

                    thread_result.points_to_keep.resize((size_t)std::ceil((T)points.size() / NB_BITS));

                    for (size_t face_index = shared_index++; face_index < faces.size(); face_index = shared_index++) {

                        const auto plane = faces[face_index]->plane();
                        T farthest_dist = epsilon;
                        size_t farthest_index = -1;

                        for (size_t point_index = 0; point_index < points.size(); point_index++) {

                            T dist = dist_to_plane(plane, points[point_index].m_point);
                            if (dist > epsilon) {
                                thread_result.points_to_keep[point_index / NB_BITS] |= (bit_type)1 << (point_index % NB_BITS);
                                if (dist > farthest_dist) {
                                    farthest_dist = dist;
                                    farthest_index = point_index;
                                }
                            }
                        }

                        if (farthest_index != -1) {
                            thread_result.farthest_indices.push_back({ faces[face_index], farthest_dist, farthest_index });
                        }
                    }
                    return thread_result;
                });

                // Aggregate the farthest points.
                auto& farthest_indices = thread_results[0].farthest_indices;
                for (size_t thread_index = 1; thread_index < thread_results.size(); thread_index++) {
                    farthest_indices.insert(farthest_indices.end(), thread_results[thread_index].farthest_indices.begin(), thread_results[thread_index].farthest_indices.end());
                }

                // Sort by index, then by distance.
                std::sort(farthest_indices.begin(), farthest_indices.end(), [](const face_it_and_farthest_index& a, const face_it_and_farthest_index& b)
                { return a.farthest_index == b.farthest_index ? a.dist > b.dist : a.farthest_index < b.farthest_index; });

                // Remove duplicate indices.
                farthest_indices.erase(std::unique(farthest_indices.begin(), farthest_indices.end(), [](const face_it_and_farthest_index& a, const face_it_and_farthest_index& b)
                { return a.farthest_index == b.farthest_index; }), farthest_indices.end());

                // Sort again, this time by distance only.
                std::sort(farthest_indices.begin(), farthest_indices.end(), [](const face_it_and_farthest_index& a, const face_it_and_farthest_index& b)
                { return a.dist > b.dist; });

                // Aggregate the points to keep.
                auto& points_to_keep = thread_results[0].points_to_keep;
                const size_t nb_buckets = points_to_keep.size();

                for (size_t thread_index = 1; thread_index < thread_results.size(); thread_index++) {
                    for (size_t bucket_index = 0; bucket_index < nb_buckets; bucket_index++) {
                        points_to_keep[bucket_index] |= thread_results[thread_index].points_to_keep[bucket_index];
                    }
                }

                // Create the result.
                struct {
                    size_t last_point_index = 0;
                    std::vector<face_it_and_farthest_point> farthest_points;
                } result;

                result.farthest_points.resize(farthest_indices.size());
                for (size_t i = 0; i < farthest_indices.size(); i++) {
                    size_t point_index = farthest_indices[i].farthest_index;
                    result.farthest_points[i].face_it = farthest_indices[i].face_it;
                    result.farthest_points[i].farthest_point = points[point_index];

                    // Mark the point as outside. If we don't end up processing it, it will be readded.
                    bit_type mask = (bit_type)1 << (point_index % NB_BITS);
                    points_to_keep[point_index / NB_BITS] &= ~mask;
                }

                // Parition the points. Important to do this after we've already collected the points, otherwise the indices will be invalid.
                result.last_point_index = 0;
                for (size_t point_index = 0; point_index < points.size(); point_index++) {
                    bit_type mask = (bit_type)1 << (point_index % NB_BITS);
                    if (points_to_keep[point_index / NB_BITS] & mask) {
                        points[result.last_point_index++] = points[point_index];
                    }
                }

                return result;
            }



            // Fill the hull with a simplex and return its center.
            template<class T, size_t N, class it>
            vecN<T, N> convex_hull<T, N, it>::initialize_to_simplex(std::span<const point_wrapper> points) {
                assert(points.size() == N + 1); // An N-d simplex consists of N+1 points!

                m_faces.clear();
                m_points.clear();

                // Add the points to the hull and calculate their center.
                std::array<point_iterator, N + 1> point_iterators;
                vecN<T, N> center;
                for (size_t i = 0; i <= N; ++i) {
                    center += points[i].m_point;
                    point_iterators[i] = insert_point(points[i]);
                }
                center /= (N + 1);

                // Create the simplex.
                std::array<face_iterator, N + 1> face_iterators;
                for (size_t i = 0; i <= N; i++) {
                    // Gather all points except the i'th one.
                    std::array<point_iterator, N> face_point_iterators;
                    size_t count = 0;
                    for (size_t j = 0; j <= N; j++) {
                        if (i != j)
                            face_point_iterators[count++] = point_iterators[j];
                    }
                    face_iterators[i] = insert_face(face_point_iterators, center);
                }

                // Set the neighbors. Each face is every other face's neighbor.
                for (size_t i = 0; i <= N; i++) {
                    size_t count = 0;
                    for (size_t j = 0; j <= N; j++) {
                        if (i != j)
                            face_iterators[i]->m_neighbors[count++] = face_iterators[j];
                    }
                }

                return center;
            }



            // Finds a large simplex that contains most points.
            // Normally returns N+1 points, unless the points are coplanar, in which case we return as many dimensions as the points allow.
            // For example in 3 dimensions:
            //  -Normally return 4 points.
            //  -If the points form a plane, return 3.
            //  -If the points for a line, return 2.
            //  -If the points are all duplicated, return 1.
            //  
            template<class T, size_t N, class it>
            std::vector<typename convex_hull<T, N, it>::point_wrapper> convex_hull<T, N, it>::find_simplex_points(std::span<const point_wrapper> extremes,
                std::span<const point_wrapper> points,
                T epsilon) {
                assert(points.size() > N);          // We need at least N+1 points to make a simplex!
                assert(extremes.size() == 2 * N);   // Inconsistent extremes size!

                std::vector<point_wrapper> simplex;
                simplex.reserve(N + 1);
                simplex.resize(2);

                // Find the two extreme points furthest appart. These will be the start of the simplex.
                T farthest_dist_sqr = 0;
                for (int index_a = 0; index_a < extremes.size(); index_a++) {
                    for (int index_b = index_a + 1; index_b < extremes.size(); index_b++) {
                        T dist_sqr = (extremes[index_a].m_point - extremes[index_b].m_point).sqr_norm();
                        if (dist_sqr > farthest_dist_sqr) {
                            farthest_dist_sqr = dist_sqr;
                            simplex[0] = extremes[index_a];
                            simplex[1] = extremes[index_b];
                        }
                    }
                }

                // Make sure the points are not all overlapping.
                if (simplex[0].m_point == simplex[1].m_point) {
                    simplex.pop_back();
                    return simplex;
                }

                // The simplex is currently 1 dimensional. Increase the dimensions by always adding the farthest point.
                auto simplex_start = simplex[0].m_point;
                std::array<vecN<T, N>, N> projection_matrix;

                while (simplex.size() <= N) {

                    // Add the new basis vector.
                    vecN<T, N> basis_vector = simplex.back().m_point - simplex_start;
                    if (simplex.size() > 2) {
                        // Orthogonalize it unless this is the first dimension.
                        vecN<T, N> basis_vector_projected;
                        for (int i = 0; i < N; i++)
                            basis_vector_projected[i] = projection_matrix[i].dot(basis_vector);
                        basis_vector -= basis_vector_projected;
                    }
                    basis_vector = basis_vector.normalized();

                    // Update the projection matrix with the new basis vector.
                    for (int i = 0; i < N; i++) {
                        for (int j = 0; j < N; j++) {
                            projection_matrix[i][j] += basis_vector[i] * basis_vector[j];
                        }
                    }

                    constexpr int CHUNK_SIZE = 1024;
                    struct index_and_dist_sqr { T dist_sqr; size_t index; };
                    auto thread_results = thread_pool::get().reserve((size_t)std::ceil((T)points.size() / CHUNK_SIZE)).dispatch_to_all([points, simplex_start, projection_matrix](size_t chunk_index) {

                        // Find the farthest point to the simplex.
                        index_and_dist_sqr farthest = { 0, 0 };

                        auto start = chunk_index * CHUNK_SIZE;
                        auto end = std::min(points.size(), (chunk_index + 1) * CHUNK_SIZE);

                        for (auto index = start; index != end; index++) {

                            // Project the point on the simplex.
                            vecN<T, N> start_to_point = points[index].m_point - simplex_start;
                            vecN<T, N> projection;
                            for (int i = 0; i < N; i++)
                                projection[i] = projection_matrix[i].dot(start_to_point);

                            // Compare with the farthest point.
                            T dist_sqr = (start_to_point - projection).sqr_norm();
                            if (dist_sqr > farthest.dist_sqr) {
                                farthest.dist_sqr = dist_sqr;
                                farthest.index = index;
                            }
                        }

                        return farthest;
                    });

                    // Aggregate the results from all threads to update the simplex.
                    auto farthest = *std::max_element(thread_results.begin(), thread_results.end(), [](const index_and_dist_sqr& a, const index_and_dist_sqr& b) { return a.dist_sqr < b.dist_sqr; });

                    if (farthest.dist_sqr < epsilon * epsilon)
                        return simplex; // The points are all coplanar.

                    simplex.push_back(points[farthest.index]);
                }

                return simplex;
            }



            // Converts a list to point_data.
            template<class T, size_t N, class it>
            template<class it2>
            std::vector<typename convex_hull<T, N, it>::point_wrapper> convex_hull<T, N, it>::convert_to_point_wrapper(it2 first, it2 last) {

                std::vector<point_wrapper> points;
                if constexpr (std::random_access_iterator<it2>)
                    points.reserve(last - first);

                // TBD is it worth multithreading this?
                while (first != last) {
                    auto& point = points.emplace_back();
                    point.m_point = *first;
                    if constexpr (!std::is_void_v<it>)
                        point.m_it = first;
                    ++first;
                }
                return points;
            }



            // Different case when the hull is empty.
            // We need to compute a large simplex first.
            // Additionally, 2d can be much more optimized.
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::initialize(std::span<point_wrapper> points) {

                // Make sure we have at least N + 1 points.
                if (points.size() <= N) {
                    m_dimensions = points.size();
                    return;
                }

                // Find the extremes.
                m_extremes = find_extremes(points);

                if constexpr (N == 2) {
                    // Optimized case for 2d initialization only. Extension cannot be optimized in the same way.
                    m_dimensions = optim_2d_initialize(points);
                }
                else {
                    // Find points on the hull that make a large simplex.
                    const auto simplex_points = find_simplex_points(m_extremes, points, epsilon());
                    m_dimensions = simplex_points.size() - 1;
                    if (m_dimensions < N)
                        return;

                    // Initialize the hull to this simplex. Use it to get a center and tolerance.
                    const auto center = initialize_to_simplex(simplex_points);

                    // Grow the hull to include all points.
                    extend_impl(points, center, epsilon());
                }

            }



            // Extends the hull to cover new points.
            // If the hull is empty, defers to initialize().
            template<class T, size_t N, class it>
            template<class it2>
                requires (std::is_void_v<it> || std::is_same_v<it, it2>)
            void convex_hull<T, N, it>::extend(it2 first, it2 last) {

                // Convert to a list of {vecN, it} so we can swap elements around without modifying the original array.
                auto points = convert_to_point_wrapper(first, last);

                // Initialization is a bit more complex.
                if (empty())
                    return initialize(points);

                // The other functions dont like dealing with no points.
                if (points.empty())
                    return;

                // Find the new extremes.
                auto prev_center = center();
                m_extremes = merge_extremes(m_extremes, find_extremes(points));

                // Extend the hull based on the old center and the new epsilon.
                extend_impl(points, prev_center, epsilon());
            }



            // Extends the hull to cover a single new point.
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::extend(vecN<T, N> point) requires std::is_void_v<it> {

                assert(!empty()); // Cannot initialize from a single point!

                // Find the new extremes.
                auto prev_center = center();
                point_wrapper point_wrapper;
                point_wrapper.m_point = point;
                for (size_t i = 0; i < N; i++) {
                    if (point_wrapper.m_point[i] < m_extremes[i].m_point[i])
                        m_extremes[i] = point_wrapper;
                    if (point_wrapper.m_point[i] < m_extremes[i + N].m_point[i])
                        m_extremes[i + N] = point_wrapper;
                }

                // Extend the hull based on the old center and the new epsilon.
                extend_impl(std::span(&point_wrapper, 1), prev_center, epsilon());
            }



            // Actual implementation of extend().
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::extend_impl(std::span<point_wrapper> points, vecN<T, N> center, T epsilon) {

                constexpr size_t TO_REMOVE = -1;

                // Initialize by checking all faces.
                std::vector<face_iterator> face_stack_to_check;
                std::vector<face_iterator> face_stack_to_remove;
                face_stack_to_check.reserve(m_faces.size());
                for (auto it = m_faces.begin(); it != m_faces.end(); ++it) {
                    face_stack_to_check.push_back(it);
                }

                // Loop until all points are inside the hull.
                while (true) {

                    // Calculate the farthest points from the hull and remove points inside the hull.
                    auto [last_point_index, farthest_points] = find_farthest_point_to_each_face(face_stack_to_check, points, epsilon);
                    points = points.subspan(0, last_point_index);
                    if (farthest_points.empty())
                        break; // There are no points left.

                    // Add each farthest point to the hull, and remove faces that can see them.
                    face_stack_to_remove.clear();
                    face_stack_to_check.clear();
                    for (auto [first_face, farthest_point] : farthest_points) {

                        // Check if the face has already been overwritten.
                        if (first_face->m_unique_index == TO_REMOVE) {
                            // Add the point back to the list and skip it.
                            points = std::span(points.data(), points.size() + 1);
                            points.back() = farthest_point;
                            continue;
                        }

                        // For both N=2 and N>2, the pipeline is
                        // 1 - Find all N-1 dimensional faces that can see the point. These will be removed.
                        // 2 - Find all N-2 dimensional edges of the faces that will be removed. Create a new face from each edge to the point.
                        // 3 - Find the neighbors for new faces.
                        // For N=2 the edges are points and there are always exactly 2. Therefore we always create 2 new faces and the neighbors are trivial.
                        size_t first_index_to_face_to_remove = face_stack_to_remove.size();
                        if constexpr (N == 2) {
                            // Implementation for N = 2 where order matters and many things can be simplified.
                            // Naming convention is a b c, where a and c are existing points and b is the new one.

                            // Check in both directions until the segments can no longer see the point.
                            face_stack_to_remove.push_back(first_face);
                            while (dist_to_plane(face_stack_to_remove.back()->m_neighbors[1]->m_plane, farthest_point.m_point) > epsilon)
                                face_stack_to_remove.push_back(face_stack_to_remove.back()->m_neighbors[1]);
                            auto point_c = face_stack_to_remove.back()->m_neighbors[1];

                            std::swap(face_stack_to_remove.back(), face_stack_to_remove[first_index_to_face_to_remove]);
                            while (dist_to_plane(face_stack_to_remove.back()->m_neighbors[0]->m_plane, farthest_point.m_point) > epsilon)
                                face_stack_to_remove.push_back(face_stack_to_remove.back()->m_neighbors[0]);
                            auto point_a = face_stack_to_remove.back(); // Because of the way we store 1 point per face in 2d, we will need to reuse this edge.
                            face_stack_to_remove.pop_back();            // We can reuse this face instead of creating a new one.

                            // Create a new segment from b to c.
                            auto point_b = m_faces.emplace(point_c);
                            point_b->m_unique_index = m_next_face_index++;
                            point_b->m_points = farthest_point;
                            point_b->m_neighbors[1] = point_c;
                            point_c->m_neighbors[0] = point_b;
                            point_b->m_plane = *plane_from_points<T, 2>(std::array{ point_b->m_points.m_point, point_c->m_points.m_point });

                            // Change the existing segment at point_a to link to point_b.
                            point_b->m_neighbors[0] = point_a;
                            point_a->m_neighbors[1] = point_b;
                            point_a->m_plane = *plane_from_points<T, 2>(std::array{ point_a->m_points.m_point, point_b->m_points.m_point });

                            // Update the faces to check and to remove.
                            face_stack_to_check.push_back(point_a);
                            face_stack_to_check.push_back(point_b);

                            auto current_faces_to_remove = std::span(face_stack_to_remove).subspan(first_index_to_face_to_remove);
                            for (auto& face_to_remove : current_faces_to_remove)
                                face_to_remove->m_unique_index = TO_REMOVE;

                        }
                        else {
                            // Implementation for N > 2 where order doesn't matter.
                            const auto compare_point_iterators = [](point_iterator a, point_iterator b) { return a->m_unique_index < b->m_unique_index; };

                            // Add the point to the hull.
                            auto farthest_point_iterator = insert_point(farthest_point);

                            // Find all faces that can see the point.
                            // The stack looks like [faces that will be removed , faces that will become edges , faces to check]
                            size_t first_index_to_face_to_make_edge = face_stack_to_remove.size();
                            size_t first_index_to_face_to_check = face_stack_to_remove.size();
                            face_stack_to_remove.push_back(first_face);
                            while (first_index_to_face_to_check < face_stack_to_remove.size()) {

                                auto current_face = face_stack_to_remove[first_index_to_face_to_check];

                                T dist = dist_to_plane(current_face->m_plane, farthest_point.m_point);
                                if (dist > epsilon) {   // TODO should this be 0?

                                    // The face can see the point. Mark it and move it to the first region.
                                    face_stack_to_remove[first_index_to_face_to_check]->m_unique_index = TO_REMOVE;
                                    std::swap(face_stack_to_remove[first_index_to_face_to_make_edge++], face_stack_to_remove[first_index_to_face_to_check]);

                                    // Add all its neighbors that aren't already on the list. 
                                    // We can shorten the search to only the new part of the stack, since previous faces will already be marked with TO_REMOVE.
                                    const auto& neighbor_faces = current_face->m_neighbors;
                                    for (const auto& neighbor_face : neighbor_faces) {
                                        if (neighbor_face->m_unique_index != TO_REMOVE &&
                                            std::find(face_stack_to_remove.begin() + first_index_to_face_to_make_edge, face_stack_to_remove.end(), neighbor_face) == face_stack_to_remove.end())
                                            face_stack_to_remove.push_back(neighbor_face);
                                    }
                                }
                                // else, the face borders a face that can see the point, so it will become an edge.
                                first_index_to_face_to_check++;
                            }
                            std::span current_faces_to_remove(face_stack_to_remove.data() + first_index_to_face_to_remove, face_stack_to_remove.data() + first_index_to_face_to_make_edge);
                            std::span current_faces_to_make_edge(face_stack_to_remove.data() + first_index_to_face_to_make_edge, face_stack_to_remove.data() + first_index_to_face_to_check);

                            assert(!current_faces_to_remove.empty() && !current_faces_to_make_edge.empty()); // The point must be added to the hull!

                            // Create a new face from each edge to the point.
                            size_t first_index_to_face_to_add = face_stack_to_check.size();
                            for (auto face_to_make_edge : current_faces_to_make_edge) {

                                for (size_t neighbor_index = 0; neighbor_index < N; neighbor_index++) {

                                    // Check if the neighbor has been removed.
                                    if (face_to_make_edge->m_neighbors[neighbor_index]->m_unique_index != TO_REMOVE)
                                        continue;

                                    // Get the points for the new face. These are every point in the edge except the neighbor_index'th one, and the new point we are adding.
                                    std::array<point_iterator, N> point_iterators;
                                    size_t count = 0;
                                    for (size_t i = 0; i < N; i++) {
                                        if (i != neighbor_index)
                                            point_iterators[count++] = face_to_make_edge->m_points[i];
                                    }
                                    assert(count == N - 1); // A new face should have N-1 points in common with its edge!
                                    point_iterators.back() = farthest_point_iterator;

                                    // Add the new face and set the edge as its last neighbor since the new point is at the back.
                                    auto new_face = face_stack_to_check.emplace_back(insert_face(point_iterators, center));
                                    new_face->m_neighbors.back() = face_to_make_edge;
                                    face_to_make_edge->m_neighbors[neighbor_index] = new_face;

                                    // Sort the points so we can easily compare them later.
                                    std::sort(new_face->m_points.begin(), new_face->m_points.end() - 1, compare_point_iterators);
                                }
                            }
                            std::span current_faces_to_add(face_stack_to_check.data() + first_index_to_face_to_add, face_stack_to_check.data() + face_stack_to_check.size());

                            assert(!current_faces_to_add.empty()); // We must add new faces!

                            // Update the neighbors for new faces.
                            static_assert(N > 2, "This neighbor finding algorithm does not work for 2d. In any case, 2d always produces only 2 new faces which are trivially each-other's neighbor.");
                            for (size_t index_a = 0; index_a < current_faces_to_add.size(); index_a++) {
                                auto face_a = current_faces_to_add[index_a];
                                size_t nb_neighbors_left = std::count(face_a->m_neighbors.begin(), face_a->m_neighbors.end() - 1, face_iterator{});

                                for (size_t index_b = index_a + 1; index_b < current_faces_to_add.size() && nb_neighbors_left > 0; index_b++) {
                                    auto face_b = current_faces_to_add[index_b];

                                    // Similar algo to std::set_intersection since the points are sorted.
                                    size_t point_index_a = 0, point_index_b = 0;
                                    size_t different_point_index_a = N - 2, different_point_index_b = N - 2; // Initialize to N-1 because the last iteration will not check anything.
                                    size_t nb_points_in_common = 0;
                                    while (point_index_a < N - 1 && point_index_b < N - 1 && abs((int)point_index_a - (int)point_index_b) < 2) {
                                        if (compare_point_iterators(face_a->m_points[point_index_a], face_b->m_points[point_index_b])) {
                                            different_point_index_a = point_index_a;
                                            point_index_a++;
                                        }
                                        else if (compare_point_iterators(face_b->m_points[point_index_b], face_a->m_points[point_index_a])) {
                                            different_point_index_b = point_index_b;
                                            point_index_b++;
                                        }
                                        else {
                                            nb_points_in_common++;
                                            point_index_a++;
                                            point_index_b++;
                                        }
                                    }

                                    // Set the neighbors.
                                    if (nb_points_in_common == N - 2) {
                                        nb_neighbors_left--;
                                        face_a->m_neighbors[different_point_index_a] = face_b;
                                        face_b->m_neighbors[different_point_index_b] = face_a;
                                    }
                                }
                                assert(nb_neighbors_left == 0); // Inconsistent neighbors!
                            }

                            assert(std::all_of(current_faces_to_remove.begin(), current_faces_to_remove.end(), [](face_iterator face) { return face->m_unique_index == TO_REMOVE; }));
                            assert(std::none_of(current_faces_to_make_edge.begin(), current_faces_to_make_edge.end(), [](face_iterator face) { return face->m_unique_index == TO_REMOVE; }));
                            assert(std::none_of(current_faces_to_add.begin(), current_faces_to_add.end(), [](face_iterator face) { return face->m_unique_index == TO_REMOVE; }));

                            // Keep only faces to remove in the stack so we can remove them later.
                            face_stack_to_remove.resize(first_index_to_face_to_make_edge);
                        }

                    }

                    // Remove TO_REMOVE faces from face_stack_to_check.
                    face_stack_to_check.erase(std::remove_if(face_stack_to_check.begin(), face_stack_to_check.end(), [](face_iterator face) { return face->m_unique_index == TO_REMOVE; }),
                        face_stack_to_check.end());

                    // Remove the faces in face_stack_to_remove.
                    for (auto face : face_stack_to_remove) {
                        if constexpr (N == 2)
                            m_faces.erase(face);
                        else
                            erase_face(face);
                    }

                }
            }



            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::optim_2d_recursion_multithread(vec_list<face>& polygon,
                std::array<std::span<point_wrapper>, 2> points,
                std::array<point_wrapper, 2> new_vertices,
                std::array<face_iterator, 2> next_vertices,
                T epsilon) requires (N == 2) {

                static_assert(N == 2, "This function should never be called with more than 2 dimensions.");

                std::array<vec_list<face>*, 2> polygons = { &polygon ,&polygon };

                auto func = [&](size_t thread_index) {
                    auto new_vertex = polygons[thread_index]->emplace(next_vertices[thread_index]);
                    new_vertex->m_points = new_vertices[thread_index];
                    optim_2d_recursion(*polygons[thread_index], points[thread_index], new_vertex, epsilon);
                };

                constexpr size_t CHUNK_SIZE = 1024;
                if (std::min(points[0].size(), points[1].size()) > CHUNK_SIZE) {

                    // There are enough remaining points to warrant different threads. Try to get threads and if not fall back to doing them one after the other.
                    if (auto threads = thread_pool::get().reserve(2); threads.full()) {

                        // vec_list is not thread-safe. Therefore we will make a temporary one for whichever contains fewer points.
                        vec_list<face> temp_polygon;

                        size_t small_index = points[0].size() > points[1].size();
                        polygons[small_index] = &temp_polygon;

                        auto insertion_vertex = next_vertices[small_index];
                        temp_polygon.emplace(temp_polygon.end(), *std::prev(insertion_vertex == polygon.begin() ? polygon.end() : insertion_vertex));
                        next_vertices[small_index] = temp_polygon.emplace(temp_polygon.end(), *insertion_vertex);

                        // Dispatch to the threads.
                        threads.dispatch_to_reserved(func);

                        // Merge temp_polygon back into polygon, skipping the first and last vertices because they are redundant.
                        temp_polygon.pop_front();
                        temp_polygon.pop_back();
                        polygon.splice(insertion_vertex, temp_polygon);
                        return;
                    }
                }

                // Do both synchronously.
                for (size_t i = 0; i < 2; i++) {
                    if (!points[i].empty())
                        func(i);
                }
            }


            // Multithreaded recursion to build a 2d convex hull.
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::optim_2d_recursion(vec_list<face>& polygon, std::span<point_wrapper> points, face_iterator vertex_b, T epsilon) requires (N == 2) {

                static_assert(N == 2, "This function should never be called with more than 2 dimensions.");
                assert(!points.empty());    // There should be at least 1 point!

                // Check if we only have the new point.
                if (points.size() == 1)
                    return;

                // Get the lines ab and bc.
                auto vertex_a = std::prev(vertex_b == polygon.begin() ? polygon.end() : vertex_b);
                auto vertex_c = std::next(vertex_b);
                auto center = (vertex_a->m_points.m_point + vertex_c->m_points.m_point) / 2;

                auto line_ab = *plane_from_points<T, 2>(std::array<vec2<T>, 2> { vertex_a->m_points.m_point, vertex_b->m_points.m_point });
                auto line_bc = *plane_from_points<T, 2>(std::array<vec2<T>, 2> { vertex_b->m_points.m_point, vertex_c->m_points.m_point });

                // TODO is this necessary?
                if (dist_to_plane(line_ab, center) > 0) line_ab = line_ab.flip();
                if (dist_to_plane(line_bc, center) > 0) line_bc = line_bc.flip();

                // Partition the points 3 way: [outside of ab, inside abc, outside bc].
                // Keep track of the farthest points in ab and bc.
                point_wrapper farthest_ab, farthest_bc;
                auto bounds = nway_partition<3>(points.begin(), points.end(), [line_ab, farthest_dist_ab = epsilon, &farthest_ab,
                    line_bc, farthest_dist_bc = epsilon, &farthest_bc,
                    epsilon](const point_wrapper& point) mutable {
                    T dist_ab = dist_to_plane(line_ab, point.m_point);
                    if (dist_ab > epsilon) {
                        if (dist_ab > farthest_dist_ab) {
                            farthest_dist_ab = dist_ab;
                            farthest_ab = point;
                        }
                        return 0;
                    }
                    T dist_bc = dist_to_plane(line_bc, point.m_point);
                    if (dist_bc > epsilon) {
                        if (dist_bc > farthest_dist_bc) {
                            farthest_dist_bc = dist_bc;
                            farthest_bc = point;
                        }
                        return 2;
                    }
                    return 1;
                });

                // Recurse in non-empty regions.
                std::span points_ab(bounds[0], bounds[1]);
                std::span points_bc(bounds[2], bounds[3]);
                optim_2d_recursion_multithread(polygon, { points_ab, points_bc }, { farthest_ab, farthest_bc }, { vertex_b, vertex_c }, epsilon);
            }

            // Given the extreme points, build a 2d convex hull.
            // Returns the dimensionality of the data. Usually 2 unless all points are colinear (1) or duplicated (0).
            template<class T, size_t N, class it>
            size_t convex_hull<T, N, it>::optim_2d_initialize(std::span<point_wrapper> points) requires (N == 2) {

                static_assert(N == 2, "This function should never be called with more than 2 dimensions.");

                m_faces.clear();

                // Find the farthest 2 points among the extremes and create a line between them.
                std::array<point_wrapper, 2> farthest_extremes;
                if ((m_extremes[0].m_point - m_extremes[2].m_point).sqr_norm() > (m_extremes[1].m_point - m_extremes[3].m_point).sqr_norm())
                    farthest_extremes = { m_extremes[0], m_extremes[2] };
                else
                    farthest_extremes = { m_extremes[1], m_extremes[3] };

                if (farthest_extremes[0].m_point == farthest_extremes[1].m_point)
                    return 0; // All points are duplicated.

                const auto line = *plane_from_points<T, 2>(std::array<vec2<T>, 2>{ farthest_extremes[0].m_point, farthest_extremes[1].m_point });


                // Partition the points above and below the line while finding the farthest points.
                std::optional<point_wrapper> farthest_pos, farthest_neg;
                auto mid = std::partition(points.begin(), points.end(), [line, dist_pos = epsilon(), dist_neg = -epsilon(), &farthest_pos, &farthest_neg](const point_wrapper& point) mutable {
                    auto dist = dist_to_plane(line, point.m_point);
                    bool pos = dist > 0;
                    if (pos) {
                        if (dist > dist_pos) {
                            dist_pos = dist;
                            farthest_pos = point;
                        }
                    }
                    else {
                        if (dist < dist_neg) {
                            dist_neg = dist;
                            farthest_neg = point;
                        }
                    }
                    return pos;
                });

                if (!farthest_pos && !farthest_neg)
                    return 1; // All points are colinear.


                // Recurse.
                std::span points_pos(points.begin(), mid);
                std::span points_neg(mid, points.end());

                auto vertex_neg = m_faces.emplace(m_faces.end()); vertex_neg->m_points = farthest_extremes[0];
                auto vertex_pos = m_faces.emplace(m_faces.end()); vertex_pos->m_points = farthest_extremes[1];

                if (!farthest_pos) {
                    farthest_pos.emplace();
                    points_pos = {};
                }
                if (!farthest_neg) {
                    farthest_neg.emplace();
                    points_neg = {};
                }
                optim_2d_recursion_multithread(m_faces, { points_pos, points_neg }, { *farthest_pos, *farthest_neg }, { vertex_pos , vertex_neg }, epsilon());

                // Finalize the faces.
                m_faces.optimize(true);
                auto current_face = m_faces.begin();
                do {
                    // Get the next face.
                    auto next_face = std::next(current_face);
                    if (next_face == m_faces.end())
                        next_face = m_faces.begin();

                    // Set the uuid, plane and neighbors.
                    current_face->m_unique_index = m_next_face_index++;
                    current_face->m_plane = *plane_from_points<T, 2>(std::array{ current_face->m_points.m_point, next_face->m_points.m_point });
                    current_face->m_neighbors[1] = next_face;
                    next_face->m_neighbors[0] = current_face;

                    current_face = next_face;
                } while (current_face != m_faces.begin());

                return 2;
            }




        } // namespace quickhull_namespace
    } // namespace details



    // Exports.
    using details::quickhull_namespace::convex_hull;
    using details::quickhull_namespace::transformed_iterator;
    using details::quickhull_namespace::make_convex_hull;



} // namespace palla