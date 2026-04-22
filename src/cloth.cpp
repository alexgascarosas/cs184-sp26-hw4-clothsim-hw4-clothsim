#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

  // Build a set for O(1) pinned lookup.
  // pinned stores entries of the form [x, y] where x is the column index and y the row index.
  unordered_set<int> pinned_set;
  for (const vector<int> &p : pinned) {
    int x = p[0];
    int y = p[1];
    pinned_set.insert(y * num_width_points + x);
  }

  // Create the grid of point masses in row-major order (x varies fastest).
  point_masses.reserve(num_width_points * num_height_points);
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      double u = (double)x / (num_width_points - 1) * width;
      double v = (double)y / (num_height_points - 1) * height;

      Vector3D pos;
      if (orientation == HORIZONTAL) {
        pos = Vector3D(u, 1.0, v);
      } else {
        // VERTICAL: small random z offset in [-1/1000, 1/1000]
        double z = ((double)rand() / RAND_MAX) * 2.0 / 1000.0 - 1.0 / 1000.0;
        pos = Vector3D(u, v, z);
      }

      bool is_pinned = pinned_set.find(y * num_width_points + x) != pinned_set.end();
      point_masses.emplace_back(pos, is_pinned);
    }
  }

  // Create springs once all masses are in place (so pointers are stable).
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];

      // Structural: left and above
      if (x > 0) {
        springs.emplace_back(pm, pm - 1, STRUCTURAL);
      }
      if (y > 0) {
        springs.emplace_back(pm, pm - num_width_points, STRUCTURAL);
      }

      // Shearing: diagonal upper-left and diagonal upper-right
      if (x > 0 && y > 0) {
        springs.emplace_back(pm, pm - num_width_points - 1, SHEARING);
      }
      if (x < num_width_points - 1 && y > 0) {
        springs.emplace_back(pm, pm - num_width_points + 1, SHEARING);
      }

      // Bending: two to the left and two above
      if (x > 1) {
        springs.emplace_back(pm, pm - 2, BENDING);
      }
      if (y > 1) {
        springs.emplace_back(pm, pm - 2 * num_width_points, BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  // External force: F_ext = m * sum(external_accelerations).
  Vector3D total_external_acc(0);
  for (const Vector3D &a : external_accelerations) {
    total_external_acc += a;
  }
  Vector3D external_force = mass * total_external_acc;

  for (PointMass &pm : point_masses) {
    pm.forces = external_force;
  }

  // Spring correction forces (Hooke's law). Bending springs use a softer ks.
  for (Spring &s : springs) {
    if (s.spring_type == STRUCTURAL && !cp->enable_structural_constraints) continue;
    if (s.spring_type == SHEARING   && !cp->enable_shearing_constraints)   continue;
    if (s.spring_type == BENDING    && !cp->enable_bending_constraints)    continue;

    double k = cp->ks;
    if (s.spring_type == BENDING) k *= 0.2;

    Vector3D delta = s.pm_a->position - s.pm_b->position;
    double dist = delta.norm();
    double f_mag = k * (dist - s.rest_length);

    Vector3D dir = delta / dist;
    Vector3D force_on_a = -f_mag * dir;
    s.pm_a->forces += force_on_a;
    s.pm_b->forces -= force_on_a;
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  double damping_factor = 1.0 - cp->damping / 100.0;
  double dt2 = delta_t * delta_t;

  for (PointMass &pm : point_masses) {
    if (pm.pinned) continue;
    Vector3D acc = pm.forces / mass;
    Vector3D new_pos = pm.position
                     + damping_factor * (pm.position - pm.last_position)
                     + acc * dt2;
    pm.last_position = pm.position;
    pm.position = new_pos;
  }

  // TODO (Part 4): Handle self-collisions.

  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.

  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

  for (Spring &s : springs) {
    Vector3D delta = s.pm_b->position - s.pm_a->position;
    double dist = delta.norm();
    double max_len = 1.1 * s.rest_length;
    if (dist <= max_len) continue;

    double overshoot = dist - max_len;
    Vector3D dir = delta / dist;

    bool a_pinned = s.pm_a->pinned;
    bool b_pinned = s.pm_b->pinned;
    if (a_pinned && b_pinned) continue;

    if (a_pinned) {
      s.pm_b->position -= overshoot * dir;
    } else if (b_pinned) {
      s.pm_a->position += overshoot * dir;
    } else {
      s.pm_a->position += 0.5 * overshoot * dir;
      s.pm_b->position -= 0.5 * overshoot * dir;
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

  for (PointMass &pm : point_masses) {
    float key = hash_position(pm.position);
    auto it = map.find(key);
    if (it == map.end()) {
      auto *bucket = new vector<PointMass *>();
      bucket->push_back(&pm);
      map[key] = bucket;
    } else {
      it->second->push_back(&pm);
    }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

  float key = hash_position(pm.position);
  auto it = map.find(key);
  if (it == map.end()) return;

  Vector3D total_correction(0);
  int count = 0;
  double min_dist = 2.0 * thickness;

  for (PointMass *candidate : *(it->second)) {
    if (candidate == &pm) continue;
    Vector3D delta = pm.position - candidate->position;
    double dist = delta.norm();
    if (dist >= min_dist) continue;

    Vector3D dir = (dist > 0) ? delta / dist : Vector3D(1, 0, 0);
    total_correction += (min_dist - dist) * dir;
    count++;
  }

  if (count == 0) return;

  Vector3D avg = total_correction / (double)count;
  pm.position += avg / simulation_steps;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  double w = 3.0 * width  / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, h);

  double bx = (pos.x - fmod(pos.x, w)) / w;
  double by = (pos.y - fmod(pos.y, h)) / h;
  double bz = (pos.z - fmod(pos.z, t)) / t;

  return (float)(bx * 31.0 + by * 31.0 * 31.0 + bz * 31.0 * 31.0 * 31.0);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
