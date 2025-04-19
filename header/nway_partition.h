#pragma once

#include <array>
#include <vector>
#include <limits>
#include <optional>
#include <utility>
#include <cassert>

namespace palla {
	namespace details {
		namespace nway_partition_namespace {



			static constexpr size_t DYNAMIC = std::numeric_limits<size_t>::max();



			// Creates an array or vector of n+1 iterators.
			template<size_t n_static = DYNAMIC, class it>
			auto create_result(size_t n_dynamic) {

				constexpr bool is_dynamic = n_static == DYNAMIC;
				using array_t = std::conditional_t<is_dynamic, std::vector<it>, std::array<it, std::max<size_t>(2, n_static + 1)>>;
				array_t result;
				if constexpr (is_dynamic)
					result.resize(std::max<size_t>(2, n_dynamic + 1));

				return result;
			}



			// Deals with edges cases like not having enough elements or regions.
			template<size_t n_static = DYNAMIC, class it>
			auto nway_partition_edge_cases(it first, it last, size_t n_dynamic) -> std::optional<std::decay_t<decltype(create_result<n_static, it>(n_dynamic))>> {

				// Empty range.
				if (first == last) {
					auto bounds = create_result<n_static, it>(n_dynamic);
					std::fill(bounds.begin(), bounds.end(), first);
					return bounds;
				}

				// No regions.
				if (n_dynamic == 0) {
					auto bounds = create_result<n_static, it>(n_dynamic);
					bounds[0] = first;
					bounds[1] = last;
					return bounds;
				}

				return std::nullopt;
			}



			// Finds the first and last incorrect elements.
			template<class it, class pred>
			std::pair<it, it> improve_bounds(it first, it last, pred&& p, size_t n_dynamic) {

				// Find the first incorrect element.
				while (first != last && p(*first) == 0)
					first++;

				if (first == last)
					return { first, last };

				// Find the last incorrect element.
				last--;
				while (first != last && p(*last) == n_dynamic - 1)
					last--;
				last++;

				return { first, last };
			}



			// Static or dynamic implementation.
			template<size_t n_static = DYNAMIC, class it, class pred>
			auto nway_partition_impl(it first, it last, pred&& p, size_t n_dynamic) {

				static_assert(std::is_integral_v<std::decay_t<decltype(p(*first))>>, "The predicate should return a number from 0 to N indicating the region the element should be placed in.");

				// Deal with edge cases.
				if (auto edge_case = nway_partition_edge_cases<n_static>(first, last, n_dynamic))
					return *edge_case;

				// Improve the bounds.
				auto [first_non_sorted, last_non_sorted] = improve_bounds(first, last, p, n_dynamic);

				// Work with reverse iterators to prevent asserts. Otherwise we get stuff like "cannot decrement before begin" even though we never actually dereference.
				// Initialize to the equivalent of [begin, end-1, ...]
				auto reverse_bounds = create_result<n_static, std::reverse_iterator<it>>(n_dynamic);
				std::fill(reverse_bounds.begin(), reverse_bounds.end(), std::make_reverse_iterator(last_non_sorted));
				if (first_non_sorted != last_non_sorted) {

					// Do the partition.
					reverse_bounds[0] = std::make_reverse_iterator(std::next(first_non_sorted));
					while (reverse_bounds[0] >= reverse_bounds[1]) {

						auto index = p(*reverse_bounds[0]);
						assert(index >= 0 && index < n_dynamic); // The function returned an invalid region.
						if (index == 0) {
							// Already in the right spot.
							if (reverse_bounds[0].base() == last) // Another assert-avoiding hack.
								break;
							else
								reverse_bounds[0]--;
						}
						else {
							// Move down all the bounds until the region.
							for (size_t i = 1; i < index; i++) {
								std::swap(*reverse_bounds[i], *reverse_bounds[i + 1]);
								reverse_bounds[i]++;
							}
							// Swap with the current one.
							if (reverse_bounds[0] >= reverse_bounds[1])
								std::swap(*reverse_bounds[0], *reverse_bounds[index]);
								reverse_bounds[index]++;
						}
					}
				}

				// Fix the bounds.
				auto bounds = create_result<n_static, it>(n_dynamic);
				bounds[0] = first;
				for (size_t i = 1; i < bounds.size(); i++)
					bounds[i] = reverse_bounds[i].base();
				bounds.back() = last;

				return bounds;
			}



			// n-way partition, static version.
			// 
			// Splits a list into several regions in-place.
			// Returns an array of iterators corresponding to the various regions.
			// Ex: [first, it1, it2, last] for nb_regions = 3, where the ranges are [first, it1), [it1, it2) and [it2, last).
			// 
			// Parameters:
			// first, last:		The begin and end iterators of the range.
			// p:				The predicate such that p(*it) is an integer in [0, nb_regions - 1] indicating the element's region.
			//
			template<int nb_regions, class it, class pred>
			auto nway_partition(it first, it last, pred&& p) { return nway_partition_impl<nb_regions>(first, last, std::forward<pred>(p), nb_regions); }



			// n-way partition, dynamic version.
			// 
			// Splits a list into several regions in-place.
			// Returns a vector of iterators corresponding to the various regions.
			// Ex: [first, it1, it2, last] for nb_regions = 3, where the ranges are [first, it1), [it1, it2) and [it2, last).
			// 
			// Parameters:
			// first, last:		The begin and end iterators of the range.
			// p:				The predicate such that p(*it) is an integer in [0, nb_regions - 1] indicating the element's region.
			// nb_regions:		The number of regions.
			//
			template<class it, class pred>
			auto nway_partition(it first, it last, pred&& p, size_t nb_regions) { return nway_partition_impl<DYNAMIC>(first, last, std::forward<pred>(p), nb_regions); }



		} // namespace nway_parition
	} // namespace details



	// Exports.
	using details::nway_partition_namespace::nway_partition;



} // namespace palla