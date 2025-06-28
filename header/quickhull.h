#pragma once

#include <ranges>

#include "geometry.h"
#include "nway_partition.h"
#include "vec_list.h"

#if __has_include("thread_pool.h")
#include "thread_pool.h"
#else
#include "fake_thread_pool.h"
#endif


namespace palla {
    namespace details {
        namespace quickhull_namespace {


            // An random access iterator for small collections.
            // The entire collection is stored inside the iterator so that its view can be destroyed and the iterators remain valid.
            // This is required for something like auto it = face.points().begin(); *it;
            // Here dereferencing it should not depend on the view returned by points().
            template<class T, size_t N>
            class self_owning_iterator {
            private:
                std::array<T, N> m_elems;
                size_t m_index = 0;

            public:
                using difference_type = std::array<T, N>::const_iterator::difference_type;
                using value_type = const T;

                // Constructor.
                self_owning_iterator() = default;
                self_owning_iterator(const std::array<T, N>& elems, size_t index) : m_elems(elems), m_index(index) {}

                // Dereferencing.
                const auto& operator*() const { return m_elems[m_index]; }
                auto* operator->() const { return &**this; }

                // Movement.
                self_owning_iterator& operator++() { ++m_index; return *this; }
                self_owning_iterator operator++(int) { auto other = *this; ++m_index; return other; }
                self_owning_iterator& operator--() { --m_index; return *this; }
                self_owning_iterator operator--(int) { auto other = *this; --m_index; return other; }
                self_owning_iterator& operator+=(difference_type i) { m_index += i; return *this; }
                self_owning_iterator& operator-=(difference_type i) { m_index -= i; return *this; }
                self_owning_iterator operator+(difference_type i) const { auto other = *this; other += i; return other; }
                self_owning_iterator operator-(difference_type i) const { auto other = *this; other -= i; return other; }
                friend self_owning_iterator operator+(difference_type i, const self_owning_iterator& it) { return *this + i; }

                // Operator[].
                auto& operator[](difference_type i) const { return *(*this + i); }

                // Operator-.
                difference_type operator-(const self_owning_iterator& other) const { assert(m_elems[0] == other.m_elems[0]); return m_index - other.m_index; }

                // Comparisons.
                auto operator==(const self_owning_iterator& other) const { assert(m_elems[0] == other.m_elems[0]); return m_index == other.m_index; }
                auto operator<=>(const self_owning_iterator& other) const { assert(m_elems[0] == other.m_elems[0]); return m_index <=> other.m_index; }
            };

            // An iterator that dereferences using a special function.
            template<class it>
            class point_dereference_iterator {
            private:
                it m_it;

            public:
                using difference_type = it::difference_type;
                using value_type = std::remove_reference_t<decltype(std::declval<it>()->point())>;

                // Constructor.
                point_dereference_iterator() = default;
                point_dereference_iterator(it it) : m_it(it) {}

                // Dereferencing.
                auto& operator*() const { return m_it->point(); }
                auto* operator->() const { return &**this; }

                // Enable various movements depending on the src iterator.
                point_dereference_iterator& operator++() requires requires { ++m_it; } { ++m_it; return *this; }
                point_dereference_iterator operator++(int) requires requires { m_it++; } { auto other = *this; ++m_it; return other; }
                point_dereference_iterator& operator--() requires requires { --m_it; } { --m_it; return *this; }
                point_dereference_iterator operator--(int) requires requires { m_it--; } { auto other = *this; --m_it; return other; }
                point_dereference_iterator& operator+=(difference_type i) requires requires { m_it += i; } { m_it += i; return *this; }
                point_dereference_iterator& operator-=(difference_type i) requires requires { m_it -= i; } { m_it -= i; return *this; }
                point_dereference_iterator operator+(difference_type i) const requires requires { m_it + i; } { return point_dereference_iterator(m_it + i); }
                point_dereference_iterator operator-(difference_type i) const requires requires { m_it - i; } { return point_dereference_iterator(m_it - i); }
                friend point_dereference_iterator operator+(difference_type i, const point_dereference_iterator& it) requires requires { i + it.m_it; } { return *this + i; }

                // Enable operator[] if the src iterator is random access.
                auto& operator[](difference_type i) const requires requires { m_it[i]; } { return *(point_dereference_iterator)(m_it + i); }

                // Enable operator- if the src iterator is random access.
                auto operator-(const point_dereference_iterator& other) const requires requires { m_it - other.m_it; } { return m_it - other.m_it; }

                // Enable comparisons.
                bool operator==(const point_dereference_iterator&) const requires std::equality_comparable<it> = default;
                auto operator<=>(const point_dereference_iterator&) const requires std::three_way_comparable<it> = default;
            };

            // An iterator that moves like src_it but is actually dst_it under the hood.
            // Say we have a std::vector<int*> and we want to make an iterator that passes through the top vector but dereferences directly to int.
            // src_it = std::vector<int*>::iterator 
            // dst_it = int*.
            // converter = [](src_it it) -> dst_it { return *it; }
            //
            // Note that this makes operators - and <=> ambiguous. Are we refering to the src or dst?
            // By default, the operators refer to src, since this is what is used in things like for(auto elem : array).
            // If bottom comparison is required, use the .base() method to access the dst iterator.
            // 
            // For example, given std::vector<int> foo = {0, 1, 2, 3, 4, 5} and std::vector<int*> bar = {&foo[1], &foo[4]}.
            // We create a sized_view with bar and the afformentioned bridge iterator.
            // Then for(auto val : sized_view) will return 1 and 4.
            //
            template<class src_it, auto converter>
            class bridge_iterator {
            private:
                src_it m_it;
                using dst_it = std::decay_t<decltype(converter(std::declval<src_it>()))>;

            public:
                using difference_type = src_it::difference_type;
                using value_type = std::iterator_traits<dst_it>::value_type;

                // Constructor.
                bridge_iterator() = default;
                bridge_iterator(src_it it) : m_it(it) {}

                // Conversion.
                dst_it base() const { return converter(m_it); }
                operator dst_it() const { return base(); }

                // Dereferencing.
                auto& operator*() const { return *static_cast<dst_it>(*this); }
                auto* operator->() const { return &**this; }

                // Enable various movements depending on the src iterator.
                bridge_iterator& operator++() requires requires { ++m_it; } { ++m_it; return *this; }
                bridge_iterator operator++(int) requires requires { m_it++; } { auto other = *this; ++m_it; return other; }
                bridge_iterator& operator--() requires requires { --m_it; } { --m_it; return *this; }
                bridge_iterator operator--(int) requires requires { m_it--; } { auto other = *this; --m_it; return other; }
                bridge_iterator& operator+=(difference_type i) requires requires { m_it += i; } { m_it += i; return *this; }
                bridge_iterator& operator-=(difference_type i) requires requires { m_it -= i; } { m_it -= i; return *this; }
                bridge_iterator operator+(difference_type i) const requires requires { m_it + i; } { return bridge_iterator(m_it + i); }
                bridge_iterator operator-(difference_type i) const requires requires { m_it - i; } { return bridge_iterator(m_it - i); }
                friend bridge_iterator operator+(difference_type i, const bridge_iterator& it) requires requires { i + it.m_it; } { return *this + i; }

                // Enable operator[] if the src iterator is random access.
                auto& operator[](difference_type i) const requires requires { m_it[i]; } { return *(bridge_iterator)(m_it + i); }

                // Enable operator- if the src iterator is random access.
                auto operator-(const bridge_iterator& other) const requires requires { m_it - other.m_it; } { return m_it - other.m_it; }

                // Enable operator- with other iterators convertible to or from dst. This allows comparing const and mutable iterators.
                template<class T>
                    requires !std::same_as<T, bridge_iterator> && std::convertible_to<T, dst_it> && requires(dst_it b) { b - b; }
                friend auto operator-(const bridge_iterator& a, const T& b) { return (dst_it)a - (dst_it)b; }
                template<class T>
                    requires !std::same_as<T, bridge_iterator> && !is_bridge_iterator_v<T> && std::convertible_to<T, dst_it> && requires(dst_it b) { b - b; }
                friend auto operator-(const T& a, const bridge_iterator& b) { return (dst_it)a - (dst_it)b; }
                template<class T>
                    requires !std::same_as<T, dst_it> && !std::same_as<T, bridge_iterator> && std::convertible_to<dst_it, T> && requires(T b) { b - b; }
                friend auto operator-(const bridge_iterator& a, const T& b) { return (T)(dst_it)a - b; }
                template<class T>
                    requires !std::same_as<T, dst_it> && !is_bridge_iterator_v<T> && !std::same_as<T, bridge_iterator> && std::convertible_to<dst_it, T> && requires(T b) { b - b; }
                friend auto operator-(const T& a, const bridge_iterator& b) { return a - (T)(dst_it)b; }

                // Enable comparisons. If the src_it is not comparable, explicitely delete them to prevent other overloads from taking over.
                bool operator==(const bridge_iterator&) const requires std::equality_comparable<src_it> = default;
                bool operator==(const bridge_iterator&) const requires !std::equality_comparable<src_it> = delete;
                auto operator<=>(const bridge_iterator&) const requires std::three_way_comparable<src_it> = default;
                auto operator<=>(const bridge_iterator&) const requires !std::three_way_comparable<src_it> = delete;


                // Enable comparisons with other iterators convertible to or from dst. This allows comparing different bridge iterators.
                template<class T>
                    requires !std::same_as<T, bridge_iterator> && std::convertible_to<T, dst_it> && std::equality_comparable<dst_it>
                friend bool operator==(const bridge_iterator& a, const T& b) { return (dst_it)a == (dst_it)b; }
                template<class T>
                    requires !std::same_as<T, bridge_iterator> && std::convertible_to<T, dst_it> && std::three_way_comparable<dst_it>
                friend auto operator<=>(const bridge_iterator& a, const T& b) { return (dst_it)a <=> (dst_it)b; }

                template<class T>
                    requires !std::same_as<T, dst_it> && !std::same_as<T, bridge_iterator> && std::convertible_to<dst_it, T> && std::equality_comparable<T>
                friend bool operator==(const bridge_iterator& a, const T& b) { return (T)(dst_it)a == b; }
                template<class T>
                    requires !std::same_as<T, dst_it> && !std::same_as<T, bridge_iterator> && std::convertible_to<dst_it, T> && std::three_way_comparable<T>
                friend auto operator<=>(const bridge_iterator& a, const T& b) { return (T)(dst_it)a <=> b; }
            };

            template<class T> struct is_bridge_iterator : public std::bool_constant<false> {};
            template<class it, auto converter> struct is_bridge_iterator<bridge_iterator<it, converter>> : public std::bool_constant<true> {};
            template<class T> constexpr bool is_bridge_iterator_v = is_bridge_iterator<T>::value;


            // A wrapper around a container. Used when the container knows its size even though the iterators dont.
            template<class container, auto converter = 0>
            class sized_view: public std::ranges::view_interface<sized_view<container, converter>> {

            private:
                const container* m_container = nullptr;
                using src_it = std::decay_t<decltype(m_container->cbegin())>;
                using it = std::conditional_t<std::is_same_v<std::decay_t<decltype(converter)>, int>, src_it, bridge_iterator<src_it, converter>>;

            public:
                sized_view() = default;
                sized_view(const container& container) : m_container(&container) {}

                it begin() const { return m_container->begin(); }
                it end() const { return m_container->end(); }
                size_t size() const { return m_container->size(); }
            };


            // Forward declare convex hull.
            template<class T, size_t N, class it>
            class convex_hull;

            template<class T, size_t N, class it>
            class convex_hull_face;

            // Point wrapper has various different forms depending on the template arguments.
            template<class T, size_t N, class it>
            struct point_wrapper_impl {
                it m_it;                            // An iterator to the original container.
                std::uint32_t m_face_count = 0;     // Used to remove points that are no longer part of any face.
                std::uint32_t m_unique_index = 0;   // A unique index used for sorting.

                vecN<T, N> point() const { return *m_it; }
                it iterator() const { return m_it; }
            };

            // In 2d, we don't need the face count and indices.
            template<class T, class it>
            struct point_wrapper_impl<T, 2, it> {
                it m_it;                            // An iterator to the original container.

                vecN<T, 2> point() const { return *m_it; }
                it iterator() const { return m_it; }
            };

            // When it is void, we store the point directly.
            template<class T, size_t N>
            struct point_wrapper_impl<T, N, void> {
                vecN<T, N> m_point;                 // A copy of the point.
                std::uint32_t m_face_count = 0;     // Used to remove points that are no longer part of any face.
                std::uint32_t m_unique_index = 0;   // A unique index used for sorting.

                const auto& point() const { return m_point; }
                void iterator() const {}
            };

            // 2d void is a special case where the point is stored directly in the faces.
            template<class T>
            struct point_wrapper_impl<T, 2, void> {
                vecN<T, 2> m_point;                 // A copy of the point.

                const auto& point() const { return m_point; }
                void iterator() const {}
            };


            // A face in the convex hull.
            template<class T, size_t N, class it>
            class convex_hull_face {

                template<class T, size_t N, class it>
                friend class convex_hull;

            private:
                using face_iterator = vec_list<convex_hull_face<T, N, it>>::iterator;
                using face_const_iterator = vec_list<convex_hull_face<T, N, it>>::const_iterator;
                using point_iterator = vec_list<point_wrapper_impl<T, N, it>>::iterator;
                using point_const_iterator = vec_list<point_wrapper_impl<T, N, it>>::const_iterator;

                size_t m_unique_index = 0;                                                                          // A unique index used for sorting.
                std::array<face_iterator, N> m_neighbors;                                                           // Iterators to neighboring faces.
                std::conditional_t<N == 2, point_wrapper_impl<T, 2, it>, std::array<point_iterator, N>> m_points;   // Iterators to points.
                planeN<T, N> m_plane;                                                                               // The face's plane, pointing outwards.

                // Required for 2d iterators to work.
                template<class it>
                friend class point_dereference_iterator;
                const auto& point() const requires (N == 2) { return m_points.point(); }

            public:
                auto plane() const { return m_plane; }

                auto points() const {
                    if constexpr (N > 2) {
                        // std::array<point_iterator> -> underlying container.
                        using container = std::decay_t<decltype(m_points)>;
                        using src_it = std::decay_t<decltype(m_points.begin())>;
                        return sized_view<container, [](src_it point) {
                            if constexpr (std::is_void_v<it>)
                                return point_dereference_iterator<point_const_iterator>(*point);
                            else
                                return (*point)->iterator();
                        }>(m_points);
                    }
                    else {
                        // std::array<face_iterator> -> point -> underlying container.
                        class point_view : public std::ranges::view_interface<point_view> {
                        private:
                            friend convex_hull_face;
                            point_view(face_const_iterator next) : m_next(next) {}

                            using src_it = self_owning_iterator<face_const_iterator, 2>;
                            using dst_it = bridge_iterator<src_it, [](src_it face) {
                                if constexpr (std::is_void_v<it>)
                                    return point_dereference_iterator<face_const_iterator>(*face);
                                else
                                    return (*face)->m_points.iterator();
                            }>;

                            face_const_iterator m_next;

                        public:
                            point_view() = default;
                            size_t size() const { return 2; }
                            dst_it begin() const { return src_it({m_next->m_neighbors[0], m_next}, 0); }
                            dst_it end() const { return begin() + 2; }
                        };
                        return point_view(m_neighbors[1]);
                    }
                }

                auto neighbors() const {
                    using container = std::decay_t<decltype(m_neighbors)>;
                    using src_it = std::decay_t<decltype(m_neighbors.begin())>;
                    return sized_view<container, [](src_it face) -> face_const_iterator { return *face; }>(m_neighbors);
                }

            };



            // An implementation of quickhull that can be extended several times.
            template<class T, size_t N, class it = void>
            class convex_hull {

                // Various sanity checks.
                static_assert(N >= 2, "A convex hull needs at least 2 dimensions.");
                static_assert(std::is_floating_point_v<T>, "T should be a floating point type, even if the underlying data are integers.");
                static_assert(std::is_void_v<it> || std::bidirectional_iterator<it>, "The iterator type should either be a bidirectional iterator or void.");

            public:
                // Public types.
                using face = convex_hull_face<T, N, it>;

            private:
                // Private types.
                using it_not_void = std::conditional_t<std::is_void_v<it>, int, it>;
                using point_wrapper = point_wrapper_impl<T, N, it>;
                using face_iterator = vec_list<face>::iterator;
                using face_const_iterator = vec_list<face>::const_iterator;
                using point_iterator = vec_list<point_wrapper>::iterator;
                using point_const_iterator = vec_list<point_wrapper>::const_iterator;

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
                template<class it2>
                void extend_unconstrained(it2 first, it2 last);
                void extend_impl(std::span<point_wrapper> points, vecN<T, N> center, T epsilon);


                // The tolerance for extending faces. If a point is closer to a face than this, it will be counted as in the face.
                T epsilon() const { return box().diagonal() * (T)1e-6; }


                // Various static helpers.
                static void                             make_face_winding_consistent(face_iterator face, vecN<T, 3> center) requires (N == 3);
                static std::array<point_wrapper, 2 * N> merge_extremes(std::span<const point_wrapper> a, std::span<const point_wrapper> b);
                static std::array<point_wrapper, 2 * N> find_extremes(std::span<const point_wrapper> points);
                static std::vector<point_wrapper>       find_simplex_points(std::span<const point_wrapper> extremes, std::span<const point_wrapper> points, T epsilon);
                static auto                             find_farthest_point_to_each_face(std::span<face_iterator> faces, std::span<point_wrapper> points, T epsilon);

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
                convex_hull(it2 first, it2 last) requires std::is_void_v<it> { extend(first, last); }
                convex_hull(it_not_void first, it_not_void last) requires !std::is_void_v<it> { extend(first, last); }


                // Reset the convex hull.
                template<class it2>
                void assign(it2 first, it2 last) requires std::is_void_v<it> { extend(first, last); }
                void assign(it_not_void first, it_not_void last) requires !std::is_void_v<it> { extend(first, last); }
                void clear() {
                    m_faces.clear();
                    m_points.clear();
                    m_extremes = {};
                    m_dimensions = 0;
                }


                // Extend the convex hull to new points.
                template<class it2>
                void extend(it2 first, it2 last) requires std::is_void_v<it> { extend_unconstrained(first, last); }
                void extend(it_not_void first, it_not_void last) requires !std::is_void_v<it> { extend_unconstrained(first, last); }

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
                        // vec_list<point_wrapper> -> underlying container.
                        return sized_view<vec_list<point_wrapper>, [](point_const_iterator point) {
                            if constexpr (std::is_void_v<it>)
                                return point_dereference_iterator<point_const_iterator>(point);
                            else
                                return point->iterator();
                        }>(m_points);
                    }
                    else {
                        // vec_list<face> -> point -> underlying container.
                        return sized_view<vec_list<face>, [](face_const_iterator face) {
                            if constexpr (std::is_void_v<it>)
                                return point_dereference_iterator<face_const_iterator>(face);
                            else
                                return face->m_points.iterator();
                        }>(m_faces);
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


            // Helper function to add a new point to the hull.
            template<class T, size_t N, class it>
            convex_hull<T, N, it>::point_iterator convex_hull<T, N, it>::insert_point(point_wrapper point, point_iterator pos) {

                static_assert(N > 2, "This function should never be called with in 2 dimensions.");

                auto point_iterator = m_points.insert(pos, point);
                point_iterator->m_unique_index = m_next_point_index++;
                return point_iterator;
            }


            // Helper function to add a new face to the hull.
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
                    points_for_plane[i] = points[i]->point();
                }

                // Compute the face's plane.
                std::optional<planeN<T, N>> plane_opt = plane_from_points<T, N>(points_for_plane);
                assert(plane_opt); // Cannot make a plane from these points!
                face->m_plane = *plane_opt;
                if (dist_to_plane(face->m_plane, center) > 0)
                    face->m_plane = face->m_plane.flip();

                return face;
            }


            // Helper function to remove a face from the hull.
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


            // Make sure faces in 3d respect the face winding.
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::make_face_winding_consistent(face_iterator face, vecN<T, 3> center) requires (N == 3) {
                assert(face->m_neighbors[0] != face_iterator{}); // The neighbors must be set before calling this function!

                auto& points = face->m_points;
                if ((points[0]->point() - points[1]->point()).cross(points[0]->point() - points[2]->point()).dot(points[0]->point() - center) < 0) {
                    std::swap(face->m_points[0], face->m_points[1]);
                    std::swap(face->m_neighbors[0], face->m_neighbors[1]);
                }
            }


            // Creates a box from a list of extreme points.
            template<class T, size_t N, class it>
            boxN<T, N> convex_hull<T, N, it>::box() const {
                boxN<T, N> box;
                for (size_t i = 0; i < N; i++) {
                    box.min[i] = m_extremes[i].point()[i];
                    box.max[i] = m_extremes[i + N].point()[i];
                }
                return box;
            }



            // Given two sets of extreme points, merge them.
            template<class T, size_t N, class it>
            std::array<typename convex_hull<T, N, it>::point_wrapper, 2 * N> convex_hull<T, N, it>::merge_extremes(std::span<const point_wrapper> a, std::span<const point_wrapper> b) {
                assert(a.size() == 2 * N && b.size() == 2 * N); // Inconsistent extremes size!

                std::array<point_wrapper, 2 * N> merge;
                for (size_t i = 0; i < N; i++) {
                    merge[i] = (a[i].point()[i] < b[i].point()[i]) ? a[i] : b[i];
                    merge[i + N] = (a[i + N].point()[i] > b[i + N].point()[i]) ? a[i + N] : b[i + N];
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
                            T val = point.point()[i];
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



            // Finds the farthest points to each face.
            // Additionally partitions the points 
            template<class T, size_t N, class it>
            auto convex_hull<T, N, it>::find_farthest_point_to_each_face(std::span<face_iterator> faces, std::span<point_wrapper> points, T epsilon) {

                // Use 1 bit per point to represent whether they point is inside out outside the hull.
                using bit_type = std::uint64_t;
                constexpr size_t NB_BITS = sizeof(bit_type) * 8;
                const size_t nb_buckets = (size_t)std::ceil((T)points.size() / NB_BITS);

                // Temporary struct.
                constexpr size_t INVALID_INDEX = -1;
                struct face_and_farthest_point_index {
                    face_iterator face;
                    size_t farthest_point_index = INVALID_INDEX;
                };

                // Dispatch to several threads.
                std::atomic<size_t> shared_index{};
                constexpr size_t CHUNK_SIZE = 1024;
                size_t nb_desired_threads = (size_t)std::ceil((T)faces.size() * points.size() / CHUNK_SIZE);
                auto thread_results = thread_pool::get().reserve(nb_desired_threads).dispatch_to_at_least_one([faces, points, epsilon, nb_buckets, &shared_index](size_t) {

                    struct {
                        std::vector<bit_type> points_to_keep;
                        std::vector<face_and_farthest_point_index> faces_and_farthest_points;
                    } result;
                    result.points_to_keep.resize(nb_buckets);

                    for (size_t face_index = shared_index++; face_index < faces.size(); face_index = shared_index++) {

                        const auto face = faces[face_index];
                        const auto plane = face->plane();
                        T farthest_dist = epsilon;
                        size_t farthest_point_index = INVALID_INDEX;

                        for (size_t point_index = 0; point_index < points.size(); point_index++) {

                            T dist = dist_to_plane(plane, points[point_index].point());
                            if (dist > epsilon) {
                                result.points_to_keep[point_index / NB_BITS] |= (bit_type)1 << (point_index % NB_BITS);
                                if (dist > farthest_dist) {
                                    farthest_dist = dist;
                                    farthest_point_index = point_index;
                                }
                            }
                        }

                        if (farthest_point_index != INVALID_INDEX) {
                            result.faces_and_farthest_points.push_back({ face, farthest_point_index });
                        }
                    }

                    std::sort(result.faces_and_farthest_points.begin(), result.faces_and_farthest_points.end(),
                        [](const face_and_farthest_point_index& a, const face_and_farthest_point_index& b) { return a.farthest_point_index < b.farthest_point_index; });
                    result.faces_and_farthest_points.push_back({ {}, INVALID_INDEX });

                    return result;
                });

                // Merge the bit arrays.
                size_t nb_threads = thread_results.size();
                size_t nb_faces_with_points = thread_results[0].faces_and_farthest_points.size();
                auto& points_to_keep = thread_results[0].points_to_keep;
                for (size_t thread_index = 1; thread_index < nb_threads; thread_index++) {
                    for (size_t bucket_index = 0; bucket_index < nb_buckets; bucket_index++) {
                        points_to_keep[bucket_index] |= thread_results[thread_index].points_to_keep[bucket_index];
                    }
                    nb_faces_with_points += thread_results[thread_index].faces_and_farthest_points.size();
                }
                nb_faces_with_points -= nb_threads;

                // Merge the farthest points.
                struct face_and_farthest_point {
                    face_iterator face;
                    point_wrapper farthest_point;
                };
                std::vector<face_and_farthest_point> faces_and_farthest_points(nb_faces_with_points);
                if (nb_threads == 1) {
                    for (size_t i = 0; i < nb_faces_with_points; i++) {
                        size_t point_index = thread_results[0].faces_and_farthest_points[i].farthest_point_index;
                        bit_type mask = (bit_type)1 << (point_index % NB_BITS);
                        points_to_keep[point_index / NB_BITS] &= ~mask;

                        faces_and_farthest_points[i].face = thread_results[0].faces_and_farthest_points[i].face;
                        faces_and_farthest_points[i].farthest_point = points[point_index];
                    }
                }
                else {
                    auto dst = faces_and_farthest_points.begin();
                    using src_it = std::vector<face_and_farthest_point_index>::const_iterator;
                    std::vector<src_it> srcs(nb_threads);
                    for (size_t i = 0; i < nb_threads; i++) {
                        srcs[i] = thread_results[i].faces_and_farthest_points.begin();
                    }
                    while (true) {
                        // Find the smallest index.
                        auto& src = *std::min_element(srcs.begin(), srcs.end(), [](const src_it& a, const src_it& b) { return a->farthest_point_index < b->farthest_point_index; });
                        if (src->farthest_point_index == -1)
                            break;

                        size_t point_index = src->farthest_point_index;
                        bit_type mask = (bit_type)1 << (point_index % NB_BITS);
                        points_to_keep[point_index / NB_BITS] &= ~mask;

                        dst->face = src->face;
                        dst->farthest_point = points[point_index];
                        dst++;
                        src++;
                    }
                }

                // Parition the points.
                size_t last_point_index = 0;
                for (size_t point_index = 0; point_index < points.size(); point_index++) {
                    bit_type mask = (bit_type)1 << (point_index % NB_BITS);
                    if (points_to_keep[point_index / NB_BITS] & mask) {
                        points[last_point_index++] = points[point_index];
                    }
                }

                struct {
                    size_t last_point_index = 0;
                    std::vector<face_and_farthest_point> faces_and_farthest_points;
                } result;
                result.last_point_index = last_point_index;
                result.faces_and_farthest_points = std::move(faces_and_farthest_points);

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
                    center += points[i].point();
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

                // For 3d, fix the face winding.
                if constexpr (N == 3) {
                    for (const auto face_iterator : face_iterators)
                        make_face_winding_consistent(face_iterator, center);
                }

                return center;
            }



            // Finds up to 2 large simplices that contains most points.
            // Normally returns N+2 points. The first N points correspond to a ring around the equator and the last 2 are the poles.
            // Two opposite simplices that meet at the equator can be made from these.
            // If there are less than N+2 points or if the points have a dimensionality lower than N, returns as many points as we can.
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
                T farthest_dist_sqr = -1;
                for (int index_a = 0; index_a < extremes.size(); index_a++) {
                    for (int index_b = index_a + 1; index_b < extremes.size(); index_b++) {
                        T dist_sqr = (extremes[index_a].point() - extremes[index_b].point()).sqr_norm();
                        if (dist_sqr > farthest_dist_sqr) {
                            farthest_dist_sqr = dist_sqr;
                            simplex[0] = extremes[index_a];
                            simplex[1] = extremes[index_b];
                        }
                    }
                }

                // Make sure the points are not all overlapping.
                if (simplex[0].point() == simplex[1].point()) {
                    simplex.pop_back();
                    return simplex;
                }

                // The simplex is currently 1 dimensional. Increase the dimensions by always adding the farthest point.
                auto simplex_start = simplex[0].point();
                std::array<vecN<T, N>, N> projection_matrix;

                constexpr int CHUNK_SIZE = 1024;
                struct index_and_dist { T dist; size_t index; };
                struct poles { index_and_dist pos, neg; };
                while (simplex.size() < N) {

                    // Add the new basis vector.
                    vecN<T, N> basis_vector = simplex.back().point() - simplex_start;
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

                    // Find the farthest point to the simplex.
                    auto thread_results = thread_pool::get().reserve((size_t)std::ceil((T)points.size() / CHUNK_SIZE)).dispatch_to_all([points, simplex_start, projection_matrix](size_t chunk_index) {

                        index_and_dist farthest = { 0, 0 };
                        auto start = chunk_index * CHUNK_SIZE;
                        auto end = std::min(points.size(), (chunk_index + 1) * CHUNK_SIZE);

                        for (auto index = start; index != end; index++) {

                            // Project the point on the simplex.
                            vecN<T, N> start_to_point = points[index].point() - simplex_start;
                            vecN<T, N> projection;
                            for (int i = 0; i < N; i++)
                                projection[i] = projection_matrix[i].dot(start_to_point);

                            // Compare with the farthest point.
                            T dist_sqr = (start_to_point - projection).sqr_norm();
                            if (dist_sqr > farthest.dist) {
                                farthest.dist = dist_sqr;
                                farthest.index = index;
                            }
                        }

                        return farthest;
                    });

                    // Aggregate the results from all threads to update the simplex.
                    auto farthest = *std::max_element(thread_results.begin(), thread_results.end(), [](const index_and_dist& a, const index_and_dist& b) { return a.dist < b.dist; });
                    if (farthest.dist < epsilon * epsilon)
                        return simplex; // The points are all coplanar.

                    simplex.push_back(points[farthest.index]);
                }

                // We now have N points splitting the space in half. Find the equator.
                std::array<vecN<T, N>, N> equatorial_points;
                for (size_t i = 0; i < N; i++)
                    equatorial_points[i] = simplex[i].point();
                auto equator = *plane_from_points<T, N>(equatorial_points);

                // Find the farthest points on both sides of the equator.
                auto thread_results = thread_pool::get().reserve((size_t)std::ceil((T)points.size() / CHUNK_SIZE)).dispatch_to_all([points, equator](size_t chunk_index) {

                    poles farthest = { {0, 0}, {0, 0} };
                    auto start = chunk_index * CHUNK_SIZE;
                    auto end = std::min(points.size(), (chunk_index + 1) * CHUNK_SIZE);

                    for (auto index = start; index != end; index++) {
                        T dist = dist_to_plane(equator, points[index].point());
                        if (dist > farthest.pos.dist) {
                            farthest.pos.dist = dist;
                            farthest.pos.index = index;
                        }
                        else if (dist < farthest.neg.dist) {
                            farthest.neg.dist = dist;
                            farthest.neg.index = index;
                        }
                    }

                    return farthest;
                });

                // Aggregate the results from all threads to update the simplex.
                auto farthest_pos = std::max_element(thread_results.begin(), thread_results.end(), [](const poles& a, const poles& b) { return a.pos.dist < b.pos.dist; })->pos;
                if(farthest_pos.dist > epsilon)
                    simplex.push_back(points[farthest_pos.index]);

                auto farthest_neg = std::min_element(thread_results.begin(), thread_results.end(), [](const poles& a, const poles& b) { return a.neg.dist < b.neg.dist; })->neg;
                if (farthest_neg.dist < epsilon)
                    simplex.push_back(points[farthest_neg.index]);

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
                    if constexpr (!std::is_void_v<it>)
                        point.m_it = first;
                    else
                        point.m_point = *first;
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
                    auto simplex_points = find_simplex_points(m_extremes, points, epsilon());
                    m_dimensions = std::clamp<size_t>(simplex_points.size() - 1, 0, N);
                    if (m_dimensions < N)
                        return;

                    // Initialize the hull to this simplex. Use it to get a center and tolerance.
                    auto center = initialize_to_simplex(std::span(simplex_points).subspan(0, N + 1));

                    // If we have 2 simplices, immediately add the remaining one since it will allow us to very quickly remove lots of points.
                    if(simplex_points.size() == N + 2)
                        extend_impl(std::span(simplex_points).subspan(N + 1, 1), center, epsilon());

                    // Grow the hull to include all points.
                    extend_impl(points, center, epsilon());
                }

            }



            // Extends the hull to cover new points.
            // If the hull is empty, defers to initialize().
            template<class T, size_t N, class it>
            template<class it2>
            void convex_hull<T, N, it>::extend_unconstrained(it2 first, it2 last) {

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
                    if (point_wrapper.point()[i] < m_extremes[i].point()[i])
                        m_extremes[i] = point_wrapper;
                    if (point_wrapper.point()[i] > m_extremes[i + N].point()[i])
                        m_extremes[i + N] = point_wrapper;
                }

                // Extend the hull based on the old center and the new epsilon.
                extend_impl(std::span(&point_wrapper, 1), prev_center, epsilon());
            }



            // Actual implementation of extend().
            template<class T, size_t N, class it>
            void convex_hull<T, N, it>::extend_impl(std::span<point_wrapper> points, vecN<T, N> center, T epsilon) {

                constexpr size_t TO_REMOVE = -1;
                class face_reverter {
                private:
                    face_iterator m_face;
                    std::decay_t<decltype(std::declval<face>().m_unique_index)> m_unique_index = 0;
                    std::decay_t<decltype(std::declval<face>().m_neighbors)> m_neighbors;
                public:
                    face_reverter() = default;
                    face_reverter(face_iterator face) : m_face(face), m_unique_index(face->m_unique_index), m_neighbors(face->m_neighbors) {}

                    void revert() const {
                        m_face->m_unique_index = m_unique_index;
                        m_face->m_neighbors = m_neighbors;
                    }
                };

                // Initialize by checking all faces.
                std::vector<face_iterator> face_stack_to_check;
                std::vector<face_iterator> face_stack_to_remove;
                std::vector<face_reverter> face_stack_to_revert;
                face_stack_to_check.reserve(m_faces.size());
                for (auto face = m_faces.begin(); face != m_faces.end(); ++face) {
                    face_stack_to_check.emplace_back() = face;
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
                    size_t farthest_point_index = 0;
                    while (farthest_point_index < farthest_points.size()) {

                        // The point might be the farthest point for many faces. Find all of them.
                        size_t start_index = farthest_point_index++;
                        auto farthest_point = farthest_points[start_index].farthest_point;
                        while (farthest_point_index < farthest_points.size() && farthest_point.point() == farthest_points[farthest_point_index].farthest_point.point()) {
                            farthest_point_index++;
                        }
                        size_t end_index = farthest_point_index;

                        // Check if any faces are not yet overwritten.
                        face_iterator first_face;
                        for (size_t index = start_index; index < end_index; index++) {
                            if (farthest_points[index].face->m_unique_index != TO_REMOVE) {
                                first_face = farthest_points[index].face;
                                break;
                            }
                        }
                        if (first_face == face_iterator{}) {
                            // All the faces for this point are already overwritten. Add the point back to the list and skip it.
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
                            while (dist_to_plane(face_stack_to_remove.back()->m_neighbors[1]->m_plane, farthest_point.point()) > epsilon)
                                face_stack_to_remove.push_back(face_stack_to_remove.back()->m_neighbors[1]);
                            auto point_c = face_stack_to_remove.back()->m_neighbors[1];

                            std::swap(face_stack_to_remove.back(), face_stack_to_remove[first_index_to_face_to_remove]);
                            while (dist_to_plane(face_stack_to_remove.back()->m_neighbors[0]->m_plane, farthest_point.point()) > epsilon)
                                face_stack_to_remove.push_back(face_stack_to_remove.back()->m_neighbors[0]);
                            auto point_a = face_stack_to_remove.back(); // Because of the way we store 1 point per face in 2d, we will need to reuse this edge.
                            face_stack_to_remove.pop_back();            // We can reuse this face instead of creating a new one.

                            // Create a new segment from b to c.
                            auto point_b = m_faces.emplace(point_c);
                            point_b->m_unique_index = m_next_face_index++;
                            point_b->m_points = farthest_point;
                            point_b->m_neighbors[1] = point_c;
                            point_c->m_neighbors[0] = point_b;
                            point_b->m_plane = *plane_from_points<T, 2>(std::array{ point_b->m_points.point(), point_c->m_points.point() });

                            // Change the existing segment at point_a to link to point_b.
                            point_b->m_neighbors[0] = point_a;
                            point_a->m_neighbors[1] = point_b;
                            point_a->m_plane = *plane_from_points<T, 2>(std::array{ point_a->m_points.point(), point_b->m_points.point() });

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
                            face_stack_to_revert.clear();
                            
                            while (first_index_to_face_to_check < face_stack_to_remove.size()) {

                                auto current_face = face_stack_to_remove[first_index_to_face_to_check];

                                T dist = dist_to_plane(current_face->m_plane, farthest_point.point());
                                if (dist > epsilon) {   // TODO should this be 0?

                                    // The face can see the point. Mark it and move it to the first region.
                                    face_stack_to_revert.emplace_back(current_face);
                                    current_face->m_unique_index = TO_REMOVE;
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

                            for (const auto current_face : current_faces_to_make_edge) {
                                face_stack_to_revert.emplace_back(current_face);
                            }
                            assert(face_stack_to_revert.size() == current_faces_to_remove.size() + current_faces_to_make_edge.size()); // Incorrect revert list!

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
                            bool do_revert = false;
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
                                if (nb_neighbors_left > 0) {
                                    // We couldnt link the edges properly. This rarely happens due to floating point precision errors.
                                    do_revert = true;
                                    break;
                                }
                            }

                            if (do_revert) {
                                // Revert modified faces.
                                for (const auto& face_to_revert : face_stack_to_revert) {
                                    face_to_revert.revert();
                                }
                                face_stack_to_remove.resize(first_index_to_face_to_remove);

                                // Remove new faces.
                                for (const auto& face_to_remove : current_faces_to_add) {
                                    erase_face(face_to_remove);
                                }
                                face_stack_to_check.resize(first_index_to_face_to_add);

                                continue;
                            }

                            // Various sanity checks.
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

                    // For 3d, fix the face winding.
                    if constexpr (N == 3) {
                        for (const auto face_iterator : face_stack_to_check)
                            make_face_winding_consistent(face_iterator, center);
                    }

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
                auto center = (vertex_a->m_points.point() + vertex_c->m_points.point()) / 2;

                auto line_ab = *plane_from_points<T, 2>(std::array<vec2<T>, 2> { vertex_a->m_points.point(), vertex_b->m_points.point() });
                auto line_bc = *plane_from_points<T, 2>(std::array<vec2<T>, 2> { vertex_b->m_points.point(), vertex_c->m_points.point() });

                // TODO is this necessary?
                if (dist_to_plane(line_ab, center) > 0) line_ab = line_ab.flip();
                if (dist_to_plane(line_bc, center) > 0) line_bc = line_bc.flip();

                // Partition the points 3 way: [outside of ab, inside abc, outside bc].
                // Keep track of the farthest points in ab and bc.
                point_wrapper farthest_ab, farthest_bc;
                auto bounds = nway_partition<3>(points.begin(), points.end(), [line_ab, farthest_dist_ab = epsilon, &farthest_ab,
                    line_bc, farthest_dist_bc = epsilon, &farthest_bc,
                    epsilon](const point_wrapper& point) mutable {
                    T dist_ab = dist_to_plane(line_ab, point.point());
                    if (dist_ab > epsilon) {
                        if (dist_ab > farthest_dist_ab) {
                            farthest_dist_ab = dist_ab;
                            farthest_ab = point;
                        }
                        return 0;
                    }
                    T dist_bc = dist_to_plane(line_bc, point.point());
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
                if ((m_extremes[0].point() - m_extremes[2].point()).sqr_norm() > (m_extremes[1].point() - m_extremes[3].point()).sqr_norm())
                    farthest_extremes = { m_extremes[0], m_extremes[2] };
                else
                    farthest_extremes = { m_extremes[1], m_extremes[3] };

                if (farthest_extremes[0].point() == farthest_extremes[1].point())
                    return 0; // All points are duplicated.

                const auto line = *plane_from_points<T, 2>(std::array<vec2<T>, 2>{ farthest_extremes[0].point(), farthest_extremes[1].point() });


                // Partition the points above and below the line while finding the farthest points.
                std::optional<point_wrapper> farthest_pos, farthest_neg;
                auto mid = std::partition(points.begin(), points.end(), [line, dist_pos = epsilon(), dist_neg = -epsilon(), &farthest_pos, &farthest_neg](const point_wrapper& point) mutable {
                    auto dist = dist_to_plane(line, point.point());
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
                    current_face->m_plane = *plane_from_points<T, 2>(std::array{ current_face->m_points.point(), next_face->m_points.point() });
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
    using details::quickhull_namespace::make_convex_hull;



} // namespace palla