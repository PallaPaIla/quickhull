# quickhull

A header-only quickhull implementation with a modern c++ interface.

## Installation and requirements

Copy the folder `header` and include `header/quickhull.h`. Requires C++ 20.

## Usage

```c++
// Create a point cloud. 
// The container can be anything with a bidirectional iterator.
// The points can be anything with an operator[] that returns a scalar.
std::vector<std::array<float, 3>> point_cloud = { 
    {0, 0, 0}, 
    {0, 0, 1}, 
    {0, 1, 0},
    {1, 0, 0} };

// Perform quickhull on the point cloud.
palla::convex_hull<float, 3> hull(point_cloud.begin(), point_cloud.end());

// Alternative helper function to deduce everything except the dimension.
auto hull = palla::make_convex_hull<3>(point_cloud);

// Various results.
auto points = hull.points(); // A view on the points of the hull.
auto faces = hull.faces();   // A view on the faces of the hull.
```

The class `convex_hull<T, N, it = void>` has 3 template arguments:
* `T` is the point scalar type, for example `float` or `double`.
* `N` is the number of dimensions.
* `it` is optional and is the iterator type of the original container. It must be either a bidirectional iterator or `void`. Using `void` stores the point coordinates directly rather than linking to the original container. This allows using different point structures and containers, but prevents associating the points back to their container.

The helper function `make_convex_hull()` can deduce the arguments automatically, including `it`.

`convex_hull` is compatible with any point structure that has an `operator[]` which returns a (reference to) scalar coordinate. For example:

```c++
// A simple 4d point.
using point_4d = std::array<float, 4>; // Has a built-in operator[].

// A complex 3d point.
struct point_3d {
    std::array<float, 3> point;
    std::array<float, 3> normal;
    std::uint8_t color = 0;
    // Need to manually overload operator[].
    float& operator[](size_t i) { return point[i]; }
    float operator[](size_t i) const { return point[i]; }
};
```

### Accessing points

Use the method `points()` to get a non-owning view of the points on the hull. The points are all unique. If `it` is `void`, they are owned by the hull and are of type `palla::vecN<T, N>`. If `it` is not `void`, they are a view on the original container.

```c++
// Create a point cloud and a convex hull.
std::vector<point_3d> point_cloud = { ... };
auto convex_hull = palla::make_convex_hull<3>(point_cloud);

// Access the unique points on the hull in a single list.
auto points = convex_hull.points();
std::cout << "The hull contains " << points.size() << " points.";
for (const auto& point : points) {
    point.normal; // The points refer back to the original ones.
    point.color;
}

// The point iterators are compatible with those of the point cloud for comparisons and subtractions.
for (auto point_it = points.begin(); point_it != points.end(); ++point_it) {
    auto point_index = point_it - point_cloud.begin();
}
```

### Accessing faces

Use the method `faces()` to get a non-owning view of the faces of the hull. Faces contain several methods:
* `points()` returns a list of `N` points. The iterators are compatible with those of `convex_hull::points()`, and if `it` is not `void`, those of the container too.
* `neighbors()` returns a list of `N` faces adjacent to this one.
* `plane()` returns a plane of the form `N * P + D = 0` where `P` is a point and `N` is the plane normal. The normal always points outside the hull.

```c++
// Create a point cloud and a convex hull.
std::vector<point_3d> point_cloud = { ... };
auto convex_hull = palla::make_convex_hull<3>(point_cloud);

// Access the faces of the hull.
std::cout << "The hull contains " << convex_hull.faces().size() << " face.\n";
for (const auto& face : convex_hull.faces()) {

    // Points.
    face.points().size(); // 3.
    for (auto point_it = face.points().begin(); point_it != face.points().end(); ++point_it) {
        point_it->color;
        auto point_index = point_it - point_cloud.begin();  
    }

    // Neighbors.
    for (int i = 0; i < 3; i++) {
        // face.neighbors() is just another list of faces.
        static_assert(std::is_same_v<decltype(face), decltype(face.neighbors()[i])>);
        // In 3d and above, face.neighbors()[i] contains all of face.points(),
        // except face.points()[i].
        auto neighbor_points = face.neighbors()[i].points();
        auto it = std::find(neighbor_points.begin(), neighbor_points.end(), face.points()[i]);
        // it == face.points().end();
    }
}
```

### Dimension-specific cases

In 2d, both the faces and points always oriented counter-clockwise, and have a predictable order:
* `faces()[i].neighbors()[0] == faces()[i - 1].neighbors()[1]`
* `points()[i] == faces()[i].points()[0] == faces()[i - 1].points()[1]`

In 3d, the face points have a consistent winding such that: <br>
`(points[0] - points[1]) × (points[0] - points[2])` points outside the hull.

### Growing the hull

New points can be added to the hull with `extend()`, which grows the hull. When `it` is not void, care is required to not invalidate the iterators. For example, simply using `push_back` on `std::vector` could invalidate iterators that are currently part of the hull.

If `it` is void, no such care is required as the hull keeps its own copy of the points. This also enables a single-point overload for `extend()`.

### Edge cases

At least `N+1` points must be provided to start an `N`-dimensional convex hull. If less points are provided, they will be discarded. This means code like this will never actually build a hull:

```c++
convex_hull<float, 3> convex_hull;
for(const auto& point : points) {
    convex_hull.extend(point);
}
convex_hull.empty(); // true!
```

If there are more than `N+1` points but they do not form an `N`-dimensional shape, the hull will not be built. This can be checked using the `dimensions()` method, which returns the actual dimensionality of the points: 0 when all points are duplicated, 1 for when they are all colinear, 2 for coplanar, etc. If `dimensions() == N`, the hull was successfully built.

### Multithreading

This implementation makes heavy use of multithreading. You can disable this by removing the header `thread_pool.h` or by calling `palla::thread_pool::get().disable()`.