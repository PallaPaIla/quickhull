#pragma once

#include <vector>
#include <optional>
#include <algorithm>
#include <concepts>
#include <cassert>

namespace palla {
    namespace details {
        namespace vec_list_namespace {


            // An std::list living inside a vector.
            // Allocates geometrically to reduce new's while still keeping every trait of std::list.
            // Keeps another (singlely-linked) list of holes when elements are erased and fills them back up later.
            // There is probably a clever way to do this using custom allocators on std::list.
            template<class T>
            class vec_list {
            private:
                // Private types.

                // Struct for elements.
                struct node {
                    node* next = nullptr;
                    node* prev = nullptr;
                    std::optional<T> elem;  // TODO optimize the optional by fudging the pointers in unused bits and directly storing a union of T and char.
                };

                // Iterators, templated for constness.
                template<class U>
                class iterator_impl {
                private:
                    // Private constructor so vec_list can create a valid iterator.
                    friend class vec_list;
                    explicit iterator_impl(node* node) : m_node(node) {}

                    // Private members.
                    node* m_node = nullptr; // The iterator is basically a wrapper around a node.

                public:
                    // Types required to satisfy std::bidirectional_iterator.
                    using difference_type = std::ptrdiff_t;
                    using value_type = U;

                    // Default constructor. The user can only create empty iterators.
                    iterator_impl() = default;

                    // Indirection.
                    U& operator*() const { return *m_node->elem; }
                    U* operator->() const { return &**this; }

                    // Increment and decrement.
                    iterator_impl& operator++() { assert(m_node->next); *this = iterator(m_node->next); return *this; }
                    iterator_impl operator++(int) { iterator_impl current = *this; ++(*this); return current; }

                    iterator_impl& operator--() { assert(m_node->prev); *this = iterator(m_node->prev); return *this; }
                    iterator_impl operator--(int) { iterator_impl current = *this; --(*this); return current; }

                    // Comparison.
                    friend bool operator==(const iterator_impl& a, const iterator_impl& b) { return a.m_node == b.m_node; }

                    // Conversion from mutable to const.
                    operator iterator_impl<const U>() const requires !std::is_const_v<U> { return iterator_impl<const U>(m_node); }

                };


                // Private members.
                std::vector<std::vector<node>> m_buckets;   // List of buckets because they are never deleted. The first bucket is always 2 elements: begin and end.
                node* m_first_hole = nullptr;               // First hole. Holes form a forward list embedded within this list. When an element is erased, it becomes the new first hole.
                node* m_last_hole = nullptr;                // Last hole. Used for splicing lists together.
                size_t m_size = 0;                          // Number of elements (not holes).
                size_t m_capacity = 0;                      // Number of elements and holes.

                // Expansion constants.
                static constexpr size_t MIN_BUCKET_SIZE = 16;
                static constexpr double GROWTH_FACTOR = 2;

                // Private functions.

                // Resizes to fit at least nb_holes new elements.
                // At the end of this function, m_first_hole should be valid.
                void resize_to_fit(std::int64_t nb_new_elements, bool is_reserve = false) {
                    auto capacity_required = m_size + nb_new_elements;
                    if (capacity_required <= m_capacity)
                        return;

                    // The new bucket should be either the minimum size or enough to fit all the required elements, whichever is larger.
                    size_t bucket_size = std::max(MIN_BUCKET_SIZE, capacity_required - m_capacity);

                    // Unless this is reserve(), we should also respect the growth factor.
                    if (!is_reserve)
                        bucket_size = std::max(bucket_size, (size_t)std::ceil(m_capacity * (GROWTH_FACTOR - 1)));

                    // Add the bucket.
                    m_capacity += bucket_size;
                    m_buckets.emplace_back(bucket_size);
                    fill_bucket_with_holes(m_buckets.size() - 1, 0);
                    assert(m_first_hole);
                }

                // Fills a bucket with holes. Used to reset buckets.
                void fill_bucket_with_holes(size_t bucket_index, size_t elem_index) {
                    assert(bucket_index > 0 && bucket_index < m_buckets.size());                    // Cannot fill bucket 0 because it contains begin and end.
                    auto& bucket = m_buckets[bucket_index];
                    assert(elem_index < bucket.size());
                    assert(m_first_hole < &bucket[elem_index] || m_first_hole > &bucket.back());    // The first hole should never be part of this bucket.

                    // Clear the bucket.
                    for (size_t i = elem_index; i < bucket.size(); i++) {
                        bucket[i].next = &bucket[i] + 1;
                        bucket[i].prev = &bucket[i] - 1;
                        bucket[i].elem = std::nullopt;
                    }

                    // Link to the first hole.
                    link_two_nodes(&bucket.back(), m_first_hole);
                    bucket[elem_index].prev = nullptr;
                    m_first_hole = &bucket[elem_index];
                    if (m_last_hole == nullptr)
                        m_last_hole = &m_buckets[bucket_index].back();
                }

                // Utility function which links prev and next.
                static void link_two_nodes(node* prev, node* next) {
                    if (next) next->prev = prev;
                    if (prev) prev->next = next;
                }

            public:
                // Public types.
                using value_type = T;
                using size_type = size_t;
                using reference = T&;
                using const_reference = const T&;
                using iterator = iterator_impl<T>;
                using const_iterator = iterator_impl<const T>;
                using reverse_iterator = std::reverse_iterator<iterator_impl<T>>;
                using const_reverse_iterator = std::reverse_iterator<iterator_impl<const T>>;


                // Public functions.

                // Constructors.
                vec_list() { m_buckets.emplace_back(2); clear(); }

                template<class it>
                    requires std::forward_iterator<it>
                vec_list(it first, it last) : vec_list() { insert(begin(), first, last); }
                vec_list(std::initializer_list<T> list) : vec_list(list.begin(), list.end()) {}

                vec_list(size_t count, const T& value) : vec_list() { insert(begin(), count, value); }
                vec_list(size_t count) : vec_list() { for (size_t i = 0; i < count; i++) insert(begin(), T{}); }

                // vec_list is always movable, even if T is not.
                vec_list(vec_list&& other) : vec_list() { *this = std::move(other); }
                vec_list& operator=(vec_list&& other) noexcept {
                    m_buckets = std::move(other.m_buckets); other.m_buckets.clear();
                    m_first_hole = std::exchange(other.m_first_hole, nullptr);
                    m_last_hole = std::exchange(other.m_last_hole, nullptr);
                    m_size = std::exchange(other.m_size, 0);
                    m_capacity = std::exchange(other.m_capacity, 0);
                    return *this;
                }

                // vec_list is copyable if T is.
                vec_list(const vec_list& other) requires std::copyable<T> : vec_list() { *this = other; }
                vec_list& operator=(const vec_list& other) requires std::copyable<T> {
                    this->clear();
                    this->insert(this->begin(), other.begin(), other.end());
                    return *this;
                }

                // Accessors.
                [[nodiscard]] bool empty() const { return m_size == 0; }
                [[nodiscard]] size_type size() const { return m_size; }
                [[nodiscard]] size_type max_size() const { return m_buckets[0].max_size(); }
                [[nodiscard]] size_type capacity() const { return m_capacity; }

                // Iterators.
                [[nodiscard]] iterator begin() { return iterator(m_buckets[0][1].next); }
                [[nodiscard]] iterator end() { return iterator(&m_buckets[0][0]); }
                [[nodiscard]] const_iterator begin() const { return const_cast<vec_list*>(this)->begin(); }
                [[nodiscard]] const_iterator end() const { return const_cast<vec_list*>(this)->end(); }
                [[nodiscard]] const_iterator cbegin() const { return begin(); }
                [[nodiscard]] const_iterator cend() const { return end(); }

                [[nodiscard]] reverse_iterator rbegin() { return std::make_reverse_iterator(end()); }
                [[nodiscard]] reverse_iterator rend() { return std::make_reverse_iterator(begin()); }
                [[nodiscard]] const_reverse_iterator rbegin() const { return std::make_reverse_iterator(end()); }
                [[nodiscard]] const_reverse_iterator rend() const { return std::make_reverse_iterator(begin()); }
                [[nodiscard]] const_reverse_iterator crbegin() const { return std::make_reverse_iterator(end()); }
                [[nodiscard]] const_reverse_iterator crend() const { return std::make_reverse_iterator(begin()); }

                // Front and back.
                [[nodiscard]] reference front() { return *begin(); }
                [[nodiscard]] const_reference front() const { return *begin(); }
                [[nodiscard]] reference back() { return *std::prev(end()); }
                [[nodiscard]] const_reference back() const { return *std::prev(end()); }

                // Assign.
                template<class it>
                void assign(it first, it last) { clear(); insert(first, last); }
                void assign(size_t count, const T& value) requires std::copyable<T> { clear(); insert(end(), count, value); }
                void assign(std::initializer_list<T> list) requires std::copyable<T> { assign(list.begin(), list.end()); }

                // Insert.
                iterator insert(const_iterator pos, T&& value) requires std::movable<T> { return emplace(pos, std::move(value)); }
                iterator insert(const_iterator pos, const T& value) requires std::copyable<T> { return emplace(pos, value); }
                void insert(const_iterator pos, std::initializer_list<T> list) requires std::copyable<T> { insert(pos, list.begin(), list.end()); }

                template<class it>
                    requires std::forward_iterator<it>
                void insert(const_iterator pos, it first, it last) {
                    resize_to_fit(std::distance(first, last));
                    while (first != last)
                        insert(pos, *(first++));
                }

                void insert(const_iterator pos, size_t count, const T& value) requires std::copyable<T> {
                    resize_to_fit(count);
                    for (size_t i = 0; i < count; i++)
                        insert(pos, value);
                }

                // Push and pop.
                template<class... Ts>
                reference emplace_back(Ts&&... args) { return *emplace(end(), std::forward<Ts>(args)...); }
                reference push_back(T&& value) requires std::movable<T> { return emplace_back(std::move(value)); }
                reference push_back(const T& value) requires std::copyable<T> { return emplace_back(value); }
                void pop_back() { erase(std::prev(end())); }

                template<class... Ts>
                reference emplace_front(Ts&&... args) { return *emplace(begin(), std::forward<Ts>(args)...); }
                reference push_front(T&& value) requires std::movable<T> { return emplace_front(std::move(value)); }
                reference push_front(const T& value) requires std::copyable<T> { return emplace_front(value); }
                void pop_front() { erase(begin()); }

                // Comparisons.
                [[nodiscard]] friend bool operator==(const vec_list& a, const vec_list& b) {
                    if (a.size() != b.size())
                        return false;
                    return std::equal(a.begin(), a.end(), b.begin(), b.end());
                }

                [[nodiscard]] friend auto operator<=>(const vec_list& a, const vec_list& b) {
                    return std::lexicographical_compare_three_way(a.begin(), a.end(), b.begin(), b.end());
                }

                // Actual emplace function that does all the work.
                template<class... Ts>
                iterator emplace(const_iterator pos, Ts&&... args) {
                    // If there are no more holes, add a new bucket to create new ones.
                    if (m_first_hole == nullptr)
                        resize_to_fit(1);

                    // Fill the first hole. If it is the last one, set the last hole to nullptr.
                    auto current = m_first_hole;
                    m_first_hole = m_first_hole->next;
                    if (m_first_hole == nullptr)
                        m_last_hole = nullptr;

                    // Set the element.
                    current->elem.emplace(std::forward<Ts>(args)...);
                    m_size++;

                    // Link the element to pos.
                    auto prev = pos.m_node->prev;
                    link_two_nodes(current, pos.m_node);
                    link_two_nodes(prev, current);
                    return iterator(current);
                }

                // Actual erase function that does all the work.
                iterator erase(const_iterator first, const_iterator last) { while (first != last) { first = erase(first); } return iterator(first.m_node); }
                iterator erase(const_iterator it) {
                    assert(it.m_node && it.m_node->elem.has_value());

                    // Erase the element.
                    m_size--;
                    it.m_node->elem = std::nullopt;

                    // Link the neighbors together.
                    auto next = it.m_node->next;
                    link_two_nodes(it.m_node->prev, it.m_node->next);
                    link_two_nodes(it.m_node, m_first_hole);

                    // Make the element the first hole.
                    m_first_hole = it.m_node;
                    if (m_last_hole == nullptr)
                        m_last_hole = m_first_hole;

                    return iterator(next);
                }

                // Clears the list.
                void clear() {
                    m_first_hole = nullptr;
                    for (size_t bucket_index = 1; bucket_index < m_buckets.size(); bucket_index++) {
                        fill_bucket_with_holes(bucket_index, 0);
                    }
                    link_two_nodes(&m_buckets[0][1], &m_buckets[0][0]);
                    if (m_buckets.size() > 1)
                        m_last_hole = &m_buckets.back().back();
                    m_size = 0;
                }

                // Reserves more memory. Much like std::vector, this bypasses geometric growth and allocates only the required amount.
                void reserve(size_t new_capacity) { resize_to_fit(new_capacity - m_capacity, true); }

                // Resizes up or down by adding or removing elements at the end.
                void resize(size_t new_size) {
                    while (m_size > new_size)
                        pop_back();
                    resize_to_fit(new_size - m_size);
                    while (m_size < new_size)
                        emplace_back();
                }

                void resize(size_t new_size, const T& value) requires std::copyable<T> {
                    while (m_size > new_size)
                        pop_back();
                    insert(end(), new_size - m_size, value);
                }

                // Reverse the list.
                void reverse() {
                    // Reverse the elements only. Holes can stay the same.
                    auto first = begin().m_node->prev;
                    auto last = end().m_node;
                    for (auto current = first->next; current != last; current = current->prev) {
                        std::swap(current->prev, current->next);
                    }
                    std::swap(first->next, last->prev);
                    first->next->prev = first;
                    last->prev->next = last;
                }

                // Splices two lists together.
                void splice(const_iterator pos, vec_list& other) {
                    assert(this != &other);
                    if (other.empty())
                        return;

                    // Insert other's buckets into this.
                    this->m_buckets.insert(m_buckets.end(), std::make_move_iterator(other.m_buckets.begin()) + 1, std::make_move_iterator(other.m_buckets.end()));
                    this->m_size += other.m_size;
                    this->m_capacity += other.m_capacity;

                    // Link the holes.
                    link_two_nodes(this->m_last_hole, other.m_first_hole);
                    this->m_last_hole = other.m_last_hole;

                    // Link the elements to pos.
                    auto prev = pos.m_node->prev;
                    link_two_nodes(other.end().m_node->prev, pos.m_node);
                    link_two_nodes(prev, other.begin().m_node);

                    // Hard reset other.
                    other = vec_list{};
                }
                void splice(const_iterator pos, vec_list&& other) { splice(pos, other); }

                // These splice functions are not optimized like they are for std::list.
                void splice(const_iterator pos, vec_list& other, const_iterator it) {
                    this->insert(pos, std::move(*iterator(it.m_node)));
                    other.erase(it);
                }
                void splice(const_iterator pos, vec_list&& other, const_iterator it) { splice(pos, other, it); }

                void splice(const_iterator pos, vec_list& other, const_iterator first, const_iterator last) {
                    if (first == other.begin() && last == other.end())
                        return splice(pos, other);
                    this->insert(pos, std::make_move_iterator(iterator(first.m_node)), std::make_move_iterator(iterator(last.m_node)));
                    other.erase(first, last);
                }
                void splice(const_iterator pos, vec_list&& other, const_iterator first, const_iterator last) { splice(pos, other, first, last); }

                // Makes the list as contiguous as possible.
                void optimize(bool shrink_to_fit) requires std::movable<T> {
                    if (m_size == 0) {
                        if (shrink_to_fit) {
                            m_buckets.resize(1);
                            m_capacity = 0;
                        }
                        clear();    // Clear organizes the holes.
                        return;
                    }

                    // Sort the buckets in ascending order.
                    std::sort(m_buckets.begin() + 1, m_buckets.end(), [](const std::vector<node>& a, const std::vector<node>& b) { return a.size() > b.size(); });

                    // Find the destination buckets. Make them just large enough to contain all the points.
                    size_t dst_capacity = 0;
                    std::vector<std::vector<node>> dst_buckets;
                    for (size_t i = 1; i < m_buckets.size() && dst_capacity < m_size; i++) {
                        // Take the bucket if:
                        // -it is below or at capacity.
                        // -it is above capacity but the next bucket is below capacity/doesnt exist.
                        size_t capacity_if_we_include_bucket = dst_capacity + m_buckets[i].size();
                        bool take_bucket = (capacity_if_we_include_bucket <= m_size) || (i + 1 == m_buckets.size()) || (dst_capacity + m_buckets[i + 1].size() < m_size);
                        if (take_bucket) {
                            dst_capacity = capacity_if_we_include_bucket;
                            dst_buckets.push_back(std::move(m_buckets[i]));
                            m_buckets[i].clear(); // move is not garanteed to clear.
                        }
                    }
                    assert(dst_capacity >= m_size);

                    // Copy over the elements.
                    auto src_node = begin().m_node;
                    auto last = end().m_node;
                    size_t dst_bucket_index = 0;
                    size_t dst_elem_index = 0;
                    auto prev = src_node->prev;
                    while (src_node != last) {
                        // Get the dst node.
                        assert(src_node->elem);
                        auto dst_node = &dst_buckets[dst_bucket_index][dst_elem_index++];
                        if (dst_elem_index == dst_buckets[dst_bucket_index].size()) {
                            dst_elem_index = 0;
                            dst_bucket_index++;
                        }
                        if (src_node != dst_node) {
                            // Swap it with the source node and update the pointers.
                            if (dst_node->elem) {
                                dst_node->prev->next = src_node;
                                dst_node->next->prev = src_node;
                            }
                            std::swap(*src_node, *dst_node);
                        }
                        link_two_nodes(prev, dst_node);
                        prev = dst_node;
                        src_node = dst_node->next;
                    }
                    link_two_nodes(prev, end().m_node);

                    // Deal with unused buckets.
                    m_first_hole = nullptr;
                    m_last_hole = nullptr;
                    if (shrink_to_fit) {
                        // Remove unused buckets.
                        m_capacity = dst_capacity;
                        m_buckets.resize(1);
                    }
                    else {
                        // Fill unused buckets with holes in reverse order so that the larger ones are used first.
                        m_buckets.erase(std::remove_if(m_buckets.begin() + 1, m_buckets.end(), [](const std::vector<node>& bucket) { return bucket.empty(); }), m_buckets.end());
                        for (size_t i = m_buckets.size() - 1; i > 0; i--) {
                            fill_bucket_with_holes(i, 0);
                        }
                    }

                    // Append the dst buckets and fill the remaining portion of the last bucket with holes.
                    m_buckets.insert(m_buckets.end(), std::make_move_iterator(dst_buckets.begin()), std::make_move_iterator(dst_buckets.end()));
                    if (dst_elem_index > 0)
                        fill_bucket_with_holes(m_buckets.size() - 1, dst_elem_index);
                }
            };


        } // namespace vec_list_namespace
    } // namespace details


    // Exports.
    using details::vec_list_namespace::vec_list;


} // namespace palla