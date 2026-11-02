#pragma once

// Mass properties as the original computes them: volume integrals over
// a polyhedron's triangles (Mirtich's method, single precision, the original's evaluation order),
// a Jacobi eigen-solver, quaternion from a rotation matrix, and the "equivalent box" of an inertia.

#include "physics/pmath.hpp"

namespace sh::phys {

class Polyhedron;

// 3x3 matrix as nine floats, m[row * 3 + col] (the original's QiMatrix3 layout).
using Mat9 = float[9];

struct PolyMass {
    float mass = 0;
    Vec3 com;        // centre of mass
    float inertia[9]{};  // about the centre of mass
};

PolyMass polyhedral_mass(const Polyhedron& poly, float density);

// The raw integrals behind polyhedral_mass (volume T0, first moments T1, second moments T2,
// products TP), plus the projection and face integrals of the last triangle, in the original's
// global-variable order (P: Pbbb Pabb Paab Paaa Pbb Pab Paa Pb Pa P1; F: Fa Fb Fc Faa Fbb Fcc Faaa
// Fbbb Fccc Faab Fbbc Fcca). For checking against the original.
struct VolumeIntegrals {
    float T0 = 0, T1[3]{}, T2[3]{}, TP[3]{};
    float P[10]{}, F[12]{};
};
VolumeIntegrals volume_integrals(const Polyhedron& poly);

// Jacobi rotations on symmetric m until off-diagonals are small; m becomes diagonal and rot
// accumulates the rotation. The game uses threshold 1e-4 and at most 16 steps.
void diagonalize(float m[9], float rot[9], float threshold, int max_steps);

// The original's QiQuat::set(QiMatrix3): note the vector part comes out negated.
Quat quat_from_matrix(const float m[9]);

// Box dimensions (full extents) with the given principal moments and mass.
Vec3 equivalent_box_size(float i0, float i1, float i2, float mass);

}  // namespace sh::phys
