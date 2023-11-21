#include <benchmark/benchmark.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

Eigen::Vector3d intersection_naive(Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d b1,
                                   Eigen::Vector3d b2) {
  Eigen::Vector3d la = a1.cross(a2);
  Eigen::Vector3d lb = b1.cross(b2);
  Eigen::Vector3d i = la.cross(lb);
  if (std::abs(i(2)) > 1e-12) {
    i = i/i(2);
  }
  return i;
}

Eigen::Vector3d intersection_const_ref(const Eigen::Vector3d& a1, const Eigen::Vector3d& a2,
                                       const Eigen::Vector3d& b1, const Eigen::Vector3d& b2) {
  Eigen::Vector3d la = a1.cross(a2);
  Eigen::Vector3d lb = b1.cross(b2);
  Eigen::Vector3d i = la.cross(lb);
  if (std::abs(i(2)) > 1e-12) {
    i = i/i(2);
  }
  return i;
}

Eigen::Vector3d intersection_eigen_ref(const Eigen::Ref<const Eigen::Vector3d>& a1,
                                       const Eigen::Ref<const Eigen::Vector3d>& a2,
                                       const Eigen::Ref<const Eigen::Vector3d>& b1,
                                       const Eigen::Ref<const Eigen::Vector3d>& b2) {
  Eigen::Vector3d la = a1.cross(a2);
  Eigen::Vector3d lb = b1.cross(b2);
  Eigen::Vector3d i = la.cross(lb);
  if (std::abs(i(2)) > 1e-12) {
    i = i/i(2);
  }
  return i;
}

static void BM_vector_intersection_naive(benchmark::State& state) {
  Eigen::Vector3d a1, a2, b1, b2, i;
  a1 << 1, 0, 1;
  a2 << 2, 0, 1;
  b1 << 0, 1, 1;
  b2 << 0, 2, 1;
  for (auto _ : state) {
    i = intersection_naive(a1, a2, b1, b2);
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(a1);
  benchmark::DoNotOptimize(a2);
  benchmark::DoNotOptimize(b1);
  benchmark::DoNotOptimize(b2);
}
BENCHMARK(BM_vector_intersection_naive);

static void BM_vector_intersection_const_ref(benchmark::State& state) {
  Eigen::Vector3d a1, a2, b1, b2, i;
  a1 << 1, 0, 1;
  a2 << 2, 0, 1;
  b1 << 0, 1, 1;
  b2 << 0, 2, 1;
  for (auto _ : state) {
    i = intersection_const_ref(a1, a2, b1, b2);
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(a1);
  benchmark::DoNotOptimize(a2);
  benchmark::DoNotOptimize(b1);
  benchmark::DoNotOptimize(b2);
}
BENCHMARK(BM_vector_intersection_const_ref);

static void BM_vector_intersection_eigen_ref(benchmark::State& state) {
  Eigen::Vector3d a1, a2, b1, b2, i;
  a1 << 1, 0, 1;
  a2 << 2, 0, 1;
  b1 << 0, 1, 1;
  b2 << 0, 2, 1;
  for (auto _ : state) {
    i = intersection_eigen_ref(a1, a2, b1, b2);
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(a1);
  benchmark::DoNotOptimize(a2);
  benchmark::DoNotOptimize(b1);
  benchmark::DoNotOptimize(b2);
}
BENCHMARK(BM_vector_intersection_eigen_ref);

static void BM_intersection_naive(benchmark::State& state) {
  Eigen::Matrix<double, 3, 4> m = Eigen::Matrix<double, 3, 4>::Random();
  Eigen::Vector3d i;
  for (auto _ : state) {
    i = intersection_naive(m.col(0), m.col(1), m.col(2), m.col(3));
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(m);
}
BENCHMARK(BM_intersection_naive);

static void BM_intersection_const_ref(benchmark::State& state) {
  Eigen::Matrix<double, 3, 4> m = Eigen::Matrix<double, 3, 4>::Random();
  Eigen::Vector3d i;
  for (auto _ : state) {
    i = intersection_const_ref(m.col(0), m.col(1), m.col(2), m.col(3));
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(m);
}
BENCHMARK(BM_intersection_const_ref);

static void BM_intersection_eigen_ref(benchmark::State& state) {
  Eigen::Matrix<double, 3, 4> m = Eigen::Matrix<double, 3, 4>::Random();
  Eigen::Vector3d i;
  for (auto _ : state) {
    i = intersection_eigen_ref(m.col(0), m.col(1), m.col(2), m.col(3));
  }
  benchmark::DoNotOptimize(i);
  benchmark::DoNotOptimize(m);
}
BENCHMARK(BM_intersection_eigen_ref);

BENCHMARK_MAIN();
