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
// A structure with more data than just 3d coordinates.
struct point_3d {
    std::array<float, 3> point;
    std::array<float, 3> normal;
    std::uint8_t color = 0;

    float& operator[](size_t i) { return point[i]; }
    float operator[](size_t i) const { return point[i]; }
    bool operator==(const point_3d&) const = default; // Not actually required.
};

// Create a point cloud and a convex hull.
std::vector<point_3d> point_cloud = { ... };
auto convex_hull = palla::make_convex_hull<3>(point_cloud);

// Access the unique points on the hull in a single list.
std::cout << "The hull contains " << convex_hull.points().size() << " points.\n";
for (const auto& point : convex_hull.points()) {
    point.normal;
    point.color;
}
for (auto point_it = convex_hull.points().begin(); point_it != convex_hull.points().end(); ++point_it) {
    auto point_index = point_it - point_cloud.begin();
}

// Access the faces of the hull.
std::cout << "The hull contains " << convex_hull.faces().size() << " face.\n";
for (auto& face : convex_hull.faces()) {
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
        std::find(neighbor_points.begin(), neighbor_points.end(), face.points()[i]);
    }
    // Plane in the form N * P + D = 0, where the normal points outwards.
    auto [normal, D] = face.plane();
```

