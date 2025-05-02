#pragma once

#include <array>
#include <algorithm>
#include <optional>
#include <span>
#include <cassert>

namespace palla {
	namespace details {
		namespace geometry_namespace {



			// Used as a shortcut in place of the macro INFINITY.
			static constexpr float infinity = std::numeric_limits<float>::infinity();



			// Some concepts we will need.
			template<class T>
			concept arithmetic = std::is_arithmetic_v<T>;

			template<class T>
			concept arithmetic_after_decay = arithmetic<std::decay_t<T>>;

			template<class T>
			concept arithmetic_array = requires(T container) {
				{ container[0] } -> arithmetic_after_decay;
			};



			// Specifies an appropriate float type for an integer.
			// i8  -> f32
			// i16 -> f32
			// f32 -> f32
			// i32 -> f64
			// i64 -> f64
			// f64 -> f64
			template<class T>
			using to_float_t = std::conditional_t<std::is_same_v<T, float> || (std::is_integral_v<T> && (sizeof(T) < 4)), float, double>;

			static_assert(std::is_same_v<to_float_t<std::int8_t>, float>, "Incorrect to_float_t implementation.");
			static_assert(std::is_same_v<to_float_t<std::int16_t>, float>, "Incorrect to_float_t implementation.");
			static_assert(std::is_same_v<to_float_t<std::int32_t>, double>, "Incorrect to_float_t implementation.");
			static_assert(std::is_same_v<to_float_t<std::int64_t>, double>, "Incorrect to_float_t implementation.");
			static_assert(std::is_same_v<to_float_t<float>, float>, "Incorrect to_float_t implementation.");
			static_assert(std::is_same_v<to_float_t<double>, double>, "Incorrect to_float_t implementation.");



			// A base class for vecN_base so we can use std::is_base_of to check if an object is a vecN_base (of potentially different type).
			// Otherwise we would need something like is_vecN<>. This is simpler.
			struct vec_marker {};



			// Implementation of an N-dimensional vector.
			template<class T, size_t N>
			requires arithmetic<T>
			struct vecN_base : public std::array<T, N>, public vec_marker {

			public:
				// Constructors.
				constexpr vecN_base() : std::array<T, N>{} {} // Vectors are zero-initialized.

				template<class... Ts>
				constexpr vecN_base(Ts... args) requires (sizeof...(Ts) == N) : 
					std::array<T, N>{(T)args...} {}

				// Conversions.
				template<class U>
				constexpr vecN_base(const U& other) requires arithmetic_array<U> && !std::is_base_of_v<vec_marker, U> {
					for (size_t i = 0; i < N; i++)
						(*this)[i] = (T)other[i]; 
				}

				template<class U>
				constexpr operator U() const requires arithmetic_array<U> {
					U other;
					using elem = std::decay_t<decltype(other[0])>;
					for (size_t i = 0; i < N; i++)
						other[i] = (elem)(*this)[i];
					return other; 
				}

				// Comparisons.
				constexpr auto operator==(const vecN_base& other) const { return (std::array<T, N>&)(*this) == (std::array<T, N>&)other; }
				constexpr auto operator<=>(const vecN_base& other) const { return (std::array<T, N>&)(*this) <=> (std::array<T, N>&)other; }

				// Assignment operators.
				template<class T2> constexpr vecN_base& operator+=(const vecN_base<T2, N>& other) { for (size_t i = 0; i < N; i++) { (*this)[i] += other[i]; } return *this; }
				template<class T2> constexpr vecN_base& operator-=(const vecN_base<T2, N>& other) { for (size_t i = 0; i < N; i++) { (*this)[i] -= other[i]; } return *this; }

				template<class T2> constexpr vecN_base& operator+=(T2 other) requires arithmetic<T2> { for (size_t i = 0; i < N; i++) { (*this)[i] += other; } return *this; }
				template<class T2> constexpr vecN_base& operator-=(T2 other) requires arithmetic<T2> { for (size_t i = 0; i < N; i++) { (*this)[i] -= other; } return *this; }
				template<class T2> constexpr vecN_base& operator*=(T2 other) requires arithmetic<T2> { for (size_t i = 0; i < N; i++) { (*this)[i] *= other; } return *this; }
				template<class T2> constexpr vecN_base& operator/=(T2 other) requires arithmetic<T2> { for (size_t i = 0; i < N; i++) { (*this)[i] /= other; } return *this; }

				// Functions.
				template<class T2, class T3 = std::common_type_t<T, T2>>
				constexpr auto dot(const vecN_base<T2, N>& other) const {
					T3 dot = 0;
					for (size_t i = 0; i < N; i++) { dot += (*this)[i] * other[i]; }
					return dot;
				}

				template<class T2, class T3 = std::common_type_t<T, T2>>
				constexpr to_float_t<T3> angle(const vecN_base<T2, N>& other) const {
					auto denom = sqrt(sqr_norm() * other.sqr_norm());
					if (denom == 0) [[unlikely]]
						return 0;
					return acos(std::clamp<T3>(dot(other) / denom, -1, 1));
				}

				constexpr auto sqr_norm() const { return dot(*this); }
				constexpr auto norm() const { return sqrt((to_float_t<T>)sqr_norm()); }

			};

			

			// Concrete specialization for n-d vectors.
			template<class T, size_t N>
			struct vecN : public vecN_base<T, N> {
				using vecN_base<T, N>::vecN_base;
				[[nodiscard]] constexpr auto normalized() const { assert(this->norm() > 0); return *this / this->norm(); }
				constexpr auto operator-() const { return *this * -1; }
			};



			// Concrete specialization for 2d vectors.
			template<class T>
			struct vecN<T, 2> : public vecN_base<T, 2> {
				using vecN_base<T, 2>::vecN_base;
				[[nodiscard]] constexpr auto normalized() const { assert(this->norm() > 0); return *this / this->norm(); }
				constexpr auto operator-() const { return *this * -1; }

				constexpr T& x() { return (*this)[0]; }
				constexpr T& y() { return (*this)[1]; }

				constexpr T x() const { return (*this)[0]; }
				constexpr T y() const { return (*this)[1]; }

				constexpr to_float_t<T> angle() const { return atan2(y(), x()); }
				constexpr auto orthogonalized() const { return vecN(-y(), x()); }

				template<class T2>
				constexpr auto cross(const vecN<T2, 2>& other) const { return x() * other.y() - y() * other.x(); }
			};



			// Concrete specialization for 3d vectors.
			template<class T>
			struct vecN<T, 3> : public vecN_base<T, 3> {
				using vecN_base<T, 3>::vecN_base;
				[[nodiscard]] constexpr auto normalized() const { assert(this->norm() > 0); return *this / this->norm(); }
				constexpr auto operator-() const { return *this * -1; }

				constexpr T& x() { return (*this)[0]; }
				constexpr T& y() { return (*this)[1]; }
				constexpr T& z() { return (*this)[2]; }

				constexpr T x() const { return (*this)[0]; }
				constexpr T y() const { return (*this)[1]; }
				constexpr T z() const { return (*this)[2]; }

				template<class T2>
				constexpr auto cross(const vecN<T2, 3>& other) const {
					using T3 = std::common_type_t<T, T2>;
					vecN<T3, 3> cross { y() * other.z() - z() * other.y(), z() * other.x() - x() * other.z(), x() * other.y() - y() * other.x() };
					return cross;
				}
			};



			// Concrete specialization for 4d vectors.
			template<class T>
			struct vecN<T, 4> : public vecN_base<T, 4> {
				using vecN_base<T, 4>::vecN_base;
				[[nodiscard]] constexpr auto normalized() const { assert(this->norm() > 0); return *this / this->norm(); }
				constexpr auto operator-() const { return *this * -1; }

				constexpr T& x() { return (*this)[0]; }
				constexpr T& y() { return (*this)[1]; }
				constexpr T& z() { return (*this)[2]; }
				constexpr T& w() { return (*this)[3]; }

				constexpr T x() const { return (*this)[0]; }
				constexpr T y() const { return (*this)[1]; }
				constexpr T z() const { return (*this)[2]; }
				constexpr T w() const { return (*this)[3]; }
			};



			// Arithmetic operators.

			// Vector + Vector = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			constexpr vecN<T3, N> operator+(const vecN<T1, N>& a, const vecN<T2, N>& b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] + b[i]; }
				return c;
			}

			// Vector - Vector = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			constexpr vecN<T3, N> operator-(const vecN<T1, N>& a, const vecN<T2, N>& b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] - b[i]; }
				return c;
			}

			// Vector * Vector = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			constexpr vecN<T3, N> operator*(const vecN<T1, N>& a, const vecN<T2, N>& b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] * b[i]; }
				return c;
			}

			// Vector / Vector = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			constexpr vecN<T3, N> operator/(const vecN<T1, N>& a, const vecN<T2, N>& b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] / b[i]; }
				return c;
			}

			// Vector + Scalar = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr vecN<T3, N> operator+(const vecN<T1, N>& a, T2 b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] + b; }
				return c;
			}

			// Vector - Scalar = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr vecN<T3, N> operator-(const vecN<T1, N>& a, T2 b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] - b; }
				return c;
			}

			// Vector * Scalar = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr vecN<T3, N> operator*(const vecN<T1, N>& a, T2 b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) { c[i] = a[i] * b; }
				return c;
			}

			// Vector / Scalar = Vector.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr vecN<T3, N> operator/(const vecN<T1, N>& a, T2 b) {
				assert(b != 0);
				vecN<T3, N> c;
				if constexpr (std::is_floating_point_v<T3>) {
					c = a * ((T3)1 / b);
				}
				else {
					for (size_t i = 0; i < N; i++) { c[i] = a[i] / b; }
				}
				return c;
			}

			// Inverse operators.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr auto operator+(T1 a, const vecN<T2, N>& b) { return b + a; }
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr auto operator-(T1 a, const vecN<T2, N>& b) { return -b + a; }
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr auto operator*(T1 a, const vecN<T2, N>& b) { return b * a; }

			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>> requires arithmetic<T1> && arithmetic<T2>
			constexpr vecN<T3, N> operator/(T1 a, const vecN<T2, N>& b) {
				vecN<T3, N> c;
				for (size_t i = 0; i < N; i++) {
					if (b[i] == 0)
						c[i] = std::copysign(infinity, b[i]);
					else
						c[i] = a / b[i];
				}
				return c;
			}



			// Planes.
			template<class T, size_t N>
			struct planeN {
				static_assert(std::is_floating_point_v<T>, "The direction cannot be an integer because it must be normalized.");

				vecN<T, N> normal;	// The direction.
				T D = 0;

				[[nodiscard]] planeN flip() const { return { -normal, -D }; }
			};



			// Rays.
			template<class T, size_t N, class F = to_float_t<T>>
			struct rayN {
				static_assert(std::is_floating_point_v<F>, "The direction cannot be an integer because it must be normalized.");

				vecN<T, N> point;	// The start point.
				vecN<F, N> dir;		// The direction.
			};



			// Boxes.
			template<class T, size_t N>
			struct boxN {
				vecN<T, N> min, max;

				constexpr auto operator<=>(const boxN&) const = default;

				constexpr auto diagonal() const { return (min - max).norm(); }
				constexpr auto sqr_diagonal() const { return (min - max).sqr_norm(); }

				template<class T2>
				[[nodiscard]] constexpr auto widen(T2 extra_dist) const {
					using T3 = std::common_type_t<T, T2>;
					return boxN<T3, N> { min - extra_dist, max + extra_dist };
				}

				template<class T2>
				[[nodiscard]] constexpr auto translate(vecN<T2, N> translation) const {
					using T3 = std::common_type_t<T, T2>;
					return boxN<T3, N> { min + translation, max + translation };
				}

				constexpr T volume() const {
					T vol = 1;
					for (size_t i = 0; i < N; i++)
						vol *= (max[i] - min[i]);
					return vol;
				}

				constexpr vecN<T, N> center() const { return (min + max) / 2; }
				constexpr vecN<T, N> size() const { return max - min; }

			};



			// Utility functions.

			// Perform gaussian or gauss-jordan elimination in-place.
			// Returns whether the operation succeeded.
			enum class elimination_type {
				upper_triangular,
				reduced_row_echelon
			};
			template<size_t nb_rows, size_t nb_cols, class container>
			constexpr void gaussian_elimination(container& matrix, elimination_type type = elimination_type::reduced_row_echelon) {

				for (size_t current_row = 0, current_col = 0; current_row < nb_rows && current_col < nb_cols; current_col++) {

					// Find the maximum row. 
					size_t max_row = current_row;
					for (size_t other_row = current_row + 1; other_row < nb_rows; other_row++) {
						if (abs(matrix[max_row][current_col]) < abs(matrix[other_row][current_col]))
							max_row = other_row;
					}

					// If the largest element in the column is 0, skip this column.
					if (matrix[max_row][current_col] == 0)
						continue; // Column of 0s.

					// Swap the max row with the current one if it is much bigger. This prevents numerical errors like division by 0.
					if (abs(matrix[max_row][current_col]) > 1000 * abs(matrix[current_row][current_col])) {
						for (size_t col = current_col; col < nb_cols; col++)
							std::swap(matrix[current_row][col], matrix[max_row][col]);
					}

					// Reduce this row.
					const auto factor = 1 / matrix[current_row][current_col];	// row row fight the power!
					matrix[current_row][current_col] = 1;
					for (size_t col = current_col + 1; col < nb_cols; col++)
						matrix[current_row][col] *= factor;

					// Reduce other rows.
					const size_t start_row = (type == elimination_type::reduced_row_echelon) ? 0 : (current_row + 1);
					for (size_t other_row = start_row; other_row < nb_rows; other_row++) {

						// Check if we need to process the row at all.
						if (other_row == current_row || matrix[other_row][current_col] == 0)
							continue;

						for (size_t col = current_col + 1; col < nb_cols; col++)
							matrix[other_row][col] -= matrix[other_row][current_col] * matrix[current_row][col];
						matrix[other_row][current_col] = 0;
					}

					// Increment the row.
					current_row++;
				}
			}



			// Inverts an NxN matrix.
			template<size_t N, class container>
			[[nodiscard]] constexpr std::optional<container> invert_matrix(const container& matrix) {

				// Concatenate the identity on the right of the matrix.
				using T = std::decay_t<decltype(matrix[0][0])>;
				std::array<std::array<T, 2 * N>, N> concatenated;
				for (size_t i = 0; i < N; i++) {
					for (size_t j = 0; j < N; j++) {
						concatenated[i][j] = matrix[i][j];
					}
					concatenated[i][i + N] = 1;
				}

				// Use gauss-jordan elimination to convert the left part into the identity.
				gaussian_elimination<N, N, elimination_type::reduced_row_echelon>(concatenated);

				// The inverse is now the right part of the concatenated matrix.
				container inverse;
				for (size_t i = 0; i < N; i++) {
					for (size_t j = 0; j < N; j++) {
						// Make sure the left is really the identity. If not, the matrix is singular.
						if (abs(concatenated[i][j] - (i == j)) > 1e-6)
							return std::nullopt;
						inverse[i][j] = concatenated[i][j + N];
					}
				}

				return inverse;
			}



			// Intersects a ray with a box. Returns the closest distance along the ray.
			template<class T, size_t N, class F = to_float_t<T>>
			std::optional<F> ray_box_intersection(vecN<T, N> ray_start, vecN<F, N> ray_dir_inv, boxN<T, N> box, F tmin = 0, F tmax = infinity) {
				for (size_t i = 0; i < N; i++) {
					F t1 = (box.min[i] - ray_start[i]) * ray_dir_inv[i];
					F t2 = (box.max[i] - ray_start[i]) * ray_dir_inv[i];
					tmin = std::max(tmin, std::min(t1, t2));
					tmax = std::min(tmax, std::max(t1, t2));
					if (tmin >= tmax)
						return std::nullopt;
				}
				return tmin;
			}



			// Intersects a ray with a box. Returns the closest distance along the ray.
			template<class T, size_t N, class F = to_float_t<T>>
			std::optional<F> ray_box_intersection(rayN<T, N> ray, boxN<T, N> box, F tmin = 0, F tmax = infinity) {
				assert(abs(ray.dir.norm() - 1) < 1e-6);
				return ray_box_intersection(ray.point, 1 / ray.dir, box, tmin, tmax);
			}



			// Returns the square distance from a point to a box. If the point is inside, returns 0.
			template<class T1, class T2, size_t N>
			auto sqr_dist_to_box(vecN<T1, N> point, boxN<T2, N> box) {
				using T3 = std::common_type_t<T1, T2>;
				T3 sqr_dist = 0;
				for (size_t i = 0; i < N; i++) {
					T3 offset = std::clamp<T3>(point[i], box.min[i], box.max[i]) - point[i];
					sqr_dist += offset * offset;
				}
				return sqr_dist;
			}



			// Returns the closest distance between 2 intervals. 
			// If the intervals intersect, returns 0.
			template<class T>
			T interval_distance(T min_a, T max_a, T min_b, T max_b) {
				return std::max({ min_a - max_b, min_b - max_b, (T)0 });
			}



			// Returns the square distance between the closest 2 points in each box.
			// If the boxes intersect, returns 0.
			template<class T1, class T2, size_t N>
			auto closest_sqr_dist_between_boxes(boxN<T1, N> a, boxN<T2, N> b) {
				using T3 = std::common_type_t<T1, T2>;
				T3 sqr_dist = 0;
				for (size_t i = 0; i < N; i++) {
					T3 min_dist = interval_distance(a.min[i], a.max[i], b.min[i], b.max[i]);
					sqr_dist += min_dist * min_dist;
				}
				return sqr_dist;
			}



			// Returns the square distance between the farthest 2 points in each box.
			template<class T1, class T2, size_t N>
			auto farthest_sqr_dist_between_boxes(boxN<T1, N> a, boxN<T2, N> b) {
				using T3 = std::common_type_t<T1, T2>;
				T3 sqr_dist = 0;
				for (size_t i = 0; i < N; i++) {
					T3 max_dist = std::max(abs(a.min[i] - b.max[i]), abs(a.max[i] - b.min[i]));
					sqr_dist += max_dist * max_dist;
				}
				return sqr_dist;
			}



			// Returns the intersection of 2 boxes, if it exists.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			std::optional<boxN<T3, N>> box_intersection(boxN<T1, N> a, boxN<T2, N> b) {
				boxN<T3, N> intersection;
				for (size_t i = 0; i < N; i++) {
					intersection.min[i] = std::max(a.min[i], b.min[i]);
					intersection.max[i] = std::min(a.max[i], b.max[i]);
					if (intersection.min[i] > intersection.max[i])
						return std::nullopt;
				}
				return intersection;
			}



			// Returns the minimal box that contains both boxes.
			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			boxN<T3, N> box_union(boxN<T1, N> a, boxN<T2, N> b) {
				boxN<T3, N> box_union;
				for (size_t i = 0; i < N; i++) {
					box_union.min[i] = std::min(a.min[i], b.min[i]);
					box_union.max[i] = std::max(a.max[i], b.max[i]);
				}
				return box_union;
			}



			// Returns whether a point is in a box.
			template<class T1, class T2, size_t N>
			bool is_point_in_box(vecN<T1, N> point, boxN<T2, N> box) {
				for (size_t i = 0; i < N; i++) {
					if (point[i] < box.min[i] || point[i] > box.max[i])
						return false;
				}
				return true;
			}



			// Returns whether the first box is fully contained in the second.
			template<class T1, class T2, size_t N>
			bool is_box_in_box(boxN<T1, N> small_box, boxN<T2, N> big_box) {
				for (size_t i = 0; i < N; i++) {
					if (small_box.min[i] < big_box.min[i] || small_box.max[i] > big_box.max[i])
						return false;
				}
				return true;
			}



			// Intersection of planes.
			template<class T, size_t N>
			vecN<T, N> intersect_planes(std::span<const planeN<T, N>> planes) {
				assert(planes.size() == N); // Need N planes to intersect at a single point.

				if constexpr (N == 2) {
					// 2d case. Solve using cross products.
					T denom = planes[0].normal.cross(planes[1].normal);
					assert(denom != 0);
					return vecN<T, 2>(planes[0].D * planes[1].normal[1] - planes[1].D * planes[0].normal[1], 
						              planes[1].D * planes[0].normal[0] - planes[0].D * planes[1].normal[0]) / denom;
				}
				else if constexpr (N == 3) {
					// 3d case. Solve using cross products.
					T denom = planes[0].normal.dot(planes[1].normal.cross(planes[2].normal));
					assert(denom != 0);
					return (planes[0].D * planes[1].normal.cross(planes[2].normal) + 
						    planes[1].D * planes[2].normal.cross(planes[0].normal) + 
						    planes[2].D * planes[0].normal.cross(planes[1].normal)) / denom;
				}
				else {
					// General case. Solve N * X = -D using matrices. 
					std::array<std::array<T, N + 1>, N> matrix;
					for (size_t i = 0; i < N; i++) {
						for (size_t j = 0; j < N; j++) {
							matrix[i][j] = planes[i].normal[j];
						}
						matrix[i][N] = planes[i].D;
					}
					bool is_solvable = gaussian_elimination<N, N + 1>(matrix);
					assert(is_solvable);
					vecN<T, N> intersection;
					for (size_t i = 0; i < N; i++)
						intersection[i] = matrix[i][N];
					return intersection;
				}
			}



			// Create an Nd plane from N points.
			template<class T, size_t N>
			std::optional<planeN<T, N>> plane_from_points(std::span<const vecN<T, N>> points) {
				assert(points.size() == N); // Need N points to define a plane.

				planeN<T, N> plane;
				if constexpr (N == 2) {
					// Directly compute the normal by swapping x and y.
					plane.normal = (points[0] - points[1]).orthogonalized();
				}
				else if constexpr (N == 3) {
					// Directly compute the normal using the cross product.
					plane.normal = (points[0] - points[1]).cross(points[0] - points[2]);
				}
				else {
					// Create an N-1 x N matrix that looks like:
					//  | Points[0] - Points[N-1] |
					//  | Points[1] - Points[N-1] |
					//  | Points[2] - Points[N-1] |
					//  |          ...            |
					// Find the nullspace by gauss jordan.
					std::array<std::array<T, N>, N - 1> matrix;
					for (size_t i = 0; i < N - 1; i++) {
						for (size_t j = 0; j < N; j++) {
							matrix[i][j] = points[i][j] - points[N - 1][j];
						}
					}
					gaussian_elimination<N - 1, N>(matrix, elimination_type::reduced_row_echelon);

					// Find the free column.
					size_t free_col = N - 1;
					for (size_t row = 0; row < N - 1; row++) {
						bool found_free_col = free_col != N - 1;
						if (matrix[row][row + found_free_col] != 1) {
							if (found_free_col)
								return std::nullopt;
							free_col = row;
						}
					}

					// Set the normal.
					for (size_t i = 0; i < N; i++) {
						plane.normal[i] = (i == free_col) ? -1 : matrix[i - (i > free_col)][free_col];
					}
				}

				if (plane.normal.sqr_norm() == 0)
					return std::nullopt;

				plane.normal = plane.normal.normalized();
				plane.D = -points[0].dot(plane.normal);

				return plane;
			}

			template<class T1, class T2, size_t N, class T3 = std::common_type_t<T1, T2>>
			T3 dist_to_plane(const planeN<T1, N>& plane, const vecN<T2, N>& point) {
				return plane.normal.dot(point) + plane.D;
			}

		} // namespace geometry_namespace



		// Private exports.

		using details::geometry_namespace::infinity;
		using details::geometry_namespace::to_float_t;

		using details::geometry_namespace::elimination_type;
		using details::geometry_namespace::gaussian_elimination;
		using details::geometry_namespace::invert_matrix;



	} // namespace details



	// Public exports.

	using details::geometry_namespace::vecN;
	template<class T> using vec2 = vecN<T, 2>;
	template<class T> using vec3 = vecN<T, 3>;
	template<class T> using vec4 = vecN<T, 4>;

	using vec2f = vec2<float>;
	using vec2d = vec2<double>;
	using vec2i = vec2<size_t>;

	using vec3f = vec3<float>;
	using vec3d = vec3<double>;
	using vec3i = vec3<size_t>;

	using vec4f = vec4<float>;
	using vec4d = vec4<double>;
	using vec4i = vec4<size_t>;


	using details::geometry_namespace::planeN;
	template<class T> using plane2 = planeN<T, 2>;
	template<class T> using plane3 = planeN<T, 3>;

	using plane2f = plane2<float>;
	using plane2d = plane2<double>;

	using plane3f = plane3<float>;
	using plane3d = plane3<double>;


	using details::geometry_namespace::rayN;
	using ray2f = rayN<float, 2>;
	using ray2d = rayN<double, 2>;

	using ray3f = rayN<float, 3>;
	using ray3d = rayN<double, 3>;


	using details::geometry_namespace::boxN;
	template<class T> using box2 = boxN<T, 2>;
	template<class T> using box3 = boxN<T, 3>;

	using box2f = box2<float>;
	using box2d = box2<double>;
	using box2i = box2<size_t>;

	using box3f = box3<float>;
	using box3d = box3<double>;
	using box3i = box3<size_t>;


	using details::geometry_namespace::box_intersection;
	using details::geometry_namespace::box_union;
	using details::geometry_namespace::closest_sqr_dist_between_boxes;
	using details::geometry_namespace::farthest_sqr_dist_between_boxes;

	using details::geometry_namespace::is_box_in_box;
	using details::geometry_namespace::is_point_in_box;

	using details::geometry_namespace::ray_box_intersection;

	using details::geometry_namespace::dist_to_plane;
	using details::geometry_namespace::plane_from_points;
	using details::geometry_namespace::intersect_planes;


} // namespace palla



// Structure bindings for vectors.
namespace std {
	template<class T, size_t N>
	struct tuple_size<palla::vecN<T, N>> : std::integral_constant<size_t, N> {};

	template<class T, size_t N, size_t i>
	struct tuple_element<i, palla::vecN<T, N>> { using type = T; };
}

template<size_t i, class T, size_t N>
const auto& get(const palla::vecN<T, N>& v) { return v[i]; }

template<size_t i, class T, size_t N>
auto& get(palla::vecN<T, N>& v) { return v[i]; }