#pragma once

#include <algorithm>
#include <array>
#include <vector>

// A class for solving the Linear Complementarity Problem (LCP)
// w = q + M * z, w^T * z = 0, w >= 0, z >= 0.  The vectors q, w, and z are
// n-tuples and the matrix M is n-by-n.  The inputs to Solve(...) are q and M.
// The outputs are w and z, which are valid when the returned bool is true but
// are invalid when the returned bool is false.
//
// The comments at the end of this file explain what the preprocessor symbol
// means regarding the LCP solver implementation.  If the algorithm fails to
// converge within the specified maximum number of iterations, consider
// increasing the number and calling Solve(...) again.

namespace crobot::math {

// Support templates for number of dimensions known at compile time or
// known only at run time.
template <typename T, int32_t... Dimensions>
class LCPSolver {};

template <typename T>
class LCPSolverShared {
protected:
  // Abstract base class construction.  A virtual destructor is not
  // provided because there are no required side effects when destroying
  // objects from the derived classes.  The member m_max_iterations is set
  // by this call to the default value n*n.
  LCPSolverShared(int32_t n)
    : m_num_iterations(0),
      m_var_basic(nullptr),
      m_var_nonbasic(nullptr),
      m_num_cols(0),
      m_augmented(nullptr),
      m_q_min(nullptr),
      m_min_ratio(nullptr),
      m_ratio(nullptr),
      m_poly(nullptr),
      m_zero((T)0),
      m_one((T)1) {
    if (n > 0) {
      m_dimension      = n;
      m_max_iterations = n * n;
    }
    else {
      m_dimension      = 0;
      m_max_iterations = 0;
    }
  }

  // Use this constructor when you need a specific representation of
  // zero and of one to be used when manipulating the polynomials of the
  // base class. In particular, this is needed to select the correct
  // zero and correct one for QFNumber objects.
  LCPSolverShared(int32_t n, T const& zero, T const& one)
    : m_num_iterations(0),
      m_var_basic(nullptr),
      m_var_nonbasic(nullptr),
      m_num_cols(0),
      m_augmented(nullptr),
      m_q_min(nullptr),
      m_min_ratio(nullptr),
      m_ratio(nullptr),
      m_poly(nullptr),
      m_zero(zero),
      m_one(one) {
    if (n > 0) {
      m_dimension      = n;
      m_max_iterations = n * n;
    }
    else {
      m_dimension      = 0;
      m_max_iterations = 0;
    }
  }

public:
  // Theoretically, when there is a solution the algorithm must converge
  // in a finite number of iterations.  The number of iterations depends
  // on the problem at hand, but we need to guard against an infinite
  // loop by limiting the number.  The implementation uses a maximum
  // number of n*n (chosen arbitrarily).  You can set the number
  // yourself, perhaps when a call to Solve fails--increase the number
  // of iterations and call and solve again.
  inline void set_max_iterations(int32_t maxIterations) {
    m_max_iterations
        = (maxIterations > 0 ? maxIterations : m_dimension * m_dimension);
  }

  inline int32_t GetMaxIterations() const {
    return m_max_iterations;
  }

  // Access the actual number of iterations used in a call to Solve.
  inline int32_t GetNumIterations() const {
    return m_num_iterations;
  }

  enum class Result
  {
    HAS_TRIVIAL_SOLUTION,
    HAS_NONTRIVIAL_SOLUTION,
    NO_SOLUTION,
    FAILED_TO_CONVERGE,
    INVALID_INPUT
  };

protected:
  // Bookkeeping of variables during the iterations of the solver.  The
  // name is either 'w' or 'z' and is used for human-readable debugging
  // help.  The 'index' is that for the original variables w[index] or
  // z[index].  The 'complementary' index is the location of the
  // complementary variable in m_var_basic[] or in m_var_nonbasic[].  The
  // 'tuple' is a pointer to &w[0] or &z[0], the choice based on name of
  // 'w' or 'z', and is used to fill in the solution values (the
  // variables are permuted during the pivoting algorithm).
  struct Variable
  {
    Variable() : name(0), index(0), complementary(0), tuple(nullptr) {
    }

    char name;
    int32_t index;
    int32_t complementary;
    T* tuple;
  };

  // The augmented problem is w = q + M*z + z[n]*U = 0, where U is an
  // n-tuple of 1-values.  We manipulate the augmented matrix
  // [M | U | p(t)] where p(t) is a column vector of polynomials of at
  // most degree n.  If p[r](t) is the polynomial for row r, then
  // p[r](0) = q[r].  These are perturbations of q[r] designed so that
  // the algorithm avoids degeneracies (a q-term becomes zero during the
  // iterations).  The basic variables are w[0] through w[n-1] and the
  // nonbasic variables are z[0] through z[n].  The returned z consists
  // only of z[0] through z[n-1].

  // The derived classes ensure that the pointers point to the correct
  // of elements for each array.  The matrix M must be stored in
  // row-major order.
  bool solve(T const* q, T const* M, T* w, T* z, Result* result) {
    // Perturb the q[r] constants to be polynomials of degree r+1
    // represented as an array of n+1 coefficients.  The coefficient
    // with index r+1 is 1 and the coefficients with indices larger
    // than r+1 are 0.
    for (int32_t r = 0; r < m_dimension; ++r) {
      m_poly[r] = &augmented(r, m_dimension + 1);
      make_zero(m_poly[r]);
      m_poly[r][0]     = q[r];
      m_poly[r][r + 1] = m_one;
    }

    // Determine whether there is the trivial solution w = z = 0.
    copy(m_poly[0], m_q_min);
    int32_t basic = 0;
    for (int32_t r = 1; r < m_dimension; ++r) {
      if (LessThan(m_poly[r], m_q_min)) {
        copy(m_poly[r], m_q_min);
        basic = r;
      }
    }

    if (!less_than_zero(m_q_min)) {
      for (int32_t r = 0; r < m_dimension; ++r) {
        w[r] = q[r];
        z[r] = m_zero;
      }

      if (result) {
        *result = Result::HAS_TRIVIAL_SOLUTION;
      }
      return true;
    }

    // Initialize the remainder of the augmented matrix with M and U.
    for (int32_t r = 0; r < m_dimension; ++r) {
      for (int32_t c = 0; c < m_dimension; ++c) {
        augmented(r, c) = M[c + m_dimension * r];
      }
      augmented(r, m_dimension) = m_one;
    }

    // Keep track of when the variables enter and exit the dictionary,
    // including where complementary variables are relocated.
    for (int32_t i = 0; i <= m_dimension; ++i) {
      m_var_basic[i].name             = 'w';
      m_var_basic[i].index            = i;
      m_var_basic[i].complementary    = i;
      m_var_basic[i].tuple            = w;
      m_var_nonbasic[i].name          = 'z';
      m_var_nonbasic[i].index         = i;
      m_var_nonbasic[i].complementary = i;
      m_var_nonbasic[i].tuple         = z;
    }

    // The augmented variable z[n] is the initial driving variable for
    // pivoting.  The equation 'basic' is the one to solve for z[n]
    // and pivoting with w[basic].  The last column of M remains all
    // 1-values for this initial step, so no algebraic computations
    // occur for M[r][n].
    int32_t driving = m_dimension;
    for (int32_t r = 0; r < m_dimension; ++r) {
      if (r != basic) {
        for (int32_t c = 0; c < m_num_cols; ++c) {
          if (c != m_dimension) {
            augmented(r, c) -= augmented(basic, c);
          }
        }
      }
    }

    for (int32_t c = 0; c < m_num_cols; ++c) {
      if (c != m_dimension) {
        augmented(basic, c) = -augmented(basic, c);
      }
    }

    m_num_iterations = 0;
    for (int32_t i = 0; i < m_max_iterations; ++i, ++m_num_iterations) {
      // The basic variable of equation 'basic' exited the
      // dictionary, so/ its complementary (nonbasic) variable must
      // become the next driving variable in order for it to enter
      // the dictionary.
      int32_t nextDriving = m_var_basic[basic].complementary;
      m_var_nonbasic[nextDriving].complementary = driving;
      std::swap(m_var_basic[basic], m_var_nonbasic[driving]);
      if (m_var_nonbasic[driving].index == m_dimension) {
        // The algorithm has converged.
        for (int32_t r = 0; r < m_dimension; ++r) {
          m_var_basic[r].tuple[m_var_basic[r].index] = m_poly[r][0];
        }
        for (int32_t c = 0; c <= m_dimension; ++c) {
          int32_t index = m_var_nonbasic[c].index;
          if (index < m_dimension) {
            m_var_nonbasic[c].tuple[index] = m_zero;
          }
        }
        if (result) {
          *result = Result::HAS_NONTRIVIAL_SOLUTION;
        }
        return true;
      }

      // Determine the 'basic' equation for which the ratio
      // -q[r]/M(r,driving) is minimized among all equations r with
      // M(r,driving) < 0.
      driving = nextDriving;
      basic   = -1;
      for (int32_t r = 0; r < m_dimension; ++r) {
        if (augmented(r, driving) < m_zero) {
          T factor = -m_one / augmented(r, driving);
          Multiply(m_poly[r], factor, m_ratio);
          if (basic == -1 || LessThan(m_ratio, m_min_ratio)) {
            copy(m_ratio, m_min_ratio);
            basic = r;
          }
        }
      }

      if (basic == -1) {
        // The coefficients of z[driving] in all the equations are
        // nonnegative, so the z[driving] variable cannot leave
        // the dictionary.  There is no solution to the LCP.
        for (int32_t r = 0; r < m_dimension; ++r) {
          w[r] = m_zero;
          z[r] = m_zero;
        }

        if (result) {
          *result = Result::NO_SOLUTION;
        }
        return false;
      }

      // Solve the basic equation so that z[driving] enters the
      // dictionary and w[basic] exits the dictionary.
      T invDenom = m_one / augmented(basic, driving);
      for (int32_t r = 0; r < m_dimension; ++r) {
        if (r != basic && augmented(r, driving) != m_zero) {
          T multiplier = augmented(r, driving) * invDenom;
          for (int32_t c = 0; c < m_num_cols; ++c) {
            if (c != driving) {
              augmented(r, c) -= augmented(basic, c) * multiplier;
            }
            else {
              augmented(r, driving) = multiplier;
            }
          }
        }
      }

      for (int32_t c = 0; c < m_num_cols; ++c) {
        if (c != driving) {
          augmented(basic, c) = -augmented(basic, c) * invDenom;
        }
        else {
          augmented(basic, driving) = invDenom;
        }
      }
    }

    // Numerical round-off errors can cause the Lemke algorithm not to
    // converge.  In particular, the code above has a test
    //   if (m_augmented[r][driving] < (T)0) { ... }
    // to determine the 'basic' equation with which to pivot.  It is
    // possible that theoretically m_augmented[r][driving]is zero but
    // rounding errors cause it to be slightly negative.  If
    // theoretically all m_augmented[r][driving] >= 0, there is no
    // solution to the LCP.  With the rounding errors, if the
    // algorithm fails to converge within the specified number of
    // iterations, NO_SOLUTION is returned, which is hopefully the
    // correct result.  It is also possible that the rounding errors
    // lead to a NO_SOLUTION (returned from inside the loop) when in
    // fact there is a solution. When the LCP solver is used by
    // intersection testing algorithms, the hope is that
    // misclassifications occur only when the two objects are nearly
    // in tangential contact.
    //
    // To determine whether the rounding errors are the problem, you
    // can execute the query using exact arithmetic with the following
    // type used for 'T' (replacing 'float' or 'double') of
    // BSRational<UIntegerAP32> Rational.
    //
    // That said, if the algorithm fails to converge and you believe
    // that the rounding errors are not causing this, please file a
    // bug report and provide the input data to the solver.

    if (result) {
      *result = Result::FAILED_TO_CONVERGE;
    }
    return false;
  }

  // Access m_augmented as a 2-dimensional array.
  inline T const& augmented(int32_t row, int32_t col) const {
    return m_augmented[col + m_num_cols * row];
  }

  inline T& augmented(int32_t row, int32_t col) {
    return m_augmented[col + m_num_cols * row];
  }

  // Support for polynomials with n+1 coefficients and degree no larger
  // than n.
  void make_zero(T* poly) {
    for (int32_t i = 0; i <= m_dimension; ++i) {
      poly[i] = m_zero;
    }
  }

  void copy(T const* poly0, T* poly1) {
    for (int32_t i = 0; i <= m_dimension; ++i) {
      poly1[i] = poly0[i];
    }
  }

  bool LessThan(T const* poly0, T const* poly1) {
    for (int32_t i = 0; i <= m_dimension; ++i) {
      if (poly0[i] < poly1[i]) {
        return true;
      }

      if (poly0[i] > poly1[i]) {
        return false;
      }
    }

    return false;
  }

  bool less_than_zero(T const* poly) {
    for (int32_t i = 0; i <= m_dimension; ++i) {
      if (poly[i] < m_zero) {
        return true;
      }

      if (poly[i] > m_zero) {
        return false;
      }
    }

    return false;
  }

  void Multiply(T const* poly, T scalar, T* product) {
    for (int32_t i = 0; i <= m_dimension; ++i) {
      product[i] = poly[i] * scalar;
    }
  }

  int32_t m_dimension;
  int32_t m_max_iterations;
  int32_t m_num_iterations;

  // These pointers are set by the derived-class constructors to arrays
  // that have the correct number of elements.  The arrays m_var_basic,
  // m_var_nonbasic, m_q_min, m_min_ratio, and m_ratio each have n+1 elements.
  // The m_augmented array has n rows and 2*(n+1) columns stored in
  // row-major order in a 1-dimensional array.  The array of pointers
  // m_poly has n elements.
  Variable* m_var_basic;
  Variable* m_var_nonbasic;
  int32_t m_num_cols;
  T* m_augmented;
  T* m_q_min;
  T* m_min_ratio;
  T* m_ratio;
  T** m_poly;
  T m_zero, m_one;
};

template <typename T, int32_t n>
class LCPSolver<T, n> : public LCPSolverShared<T> {
public:
  // Construction.  The member m_max_iterations is set by this call to the
  // default value n*n.
  LCPSolver() : LCPSolverShared<T>(n) {
    this->m_var_basic    = m_array_var_basic.data();
    this->m_var_nonbasic = m_array_var_nonbasic.data();
    this->m_num_cols     = 2 * (n + 1);
    this->m_augmented    = m_array_augmented.data();
    this->m_q_min        = m_array_q_min.data();
    this->m_min_ratio    = m_array_min_ratio.data();
    this->m_ratio        = m_array_ratio.data();
    this->m_poly         = m_array_poly.data();
  }

  // Use this constructor when you need a specific representation of
  // zero and of one to be used when manipulating the polynomials of the
  // base class. In particular, this is needed to select the correct
  // zero and correct one for QFNumber objects.
  LCPSolver(T const& zero, T const& one) : LCPSolverShared<T>(n, zero, one) {
    this->m_var_basic    = m_array_var_basic.data();
    this->m_var_nonbasic = m_array_var_nonbasic.data();
    this->m_num_cols     = 2 * (n + 1);
    this->m_augmented    = m_array_augmented.data();
    this->m_q_min        = m_array_q_min.data();
    this->m_min_ratio    = m_array_min_ratio.data();
    this->m_ratio        = m_array_ratio.data();
    this->m_poly         = m_array_poly.data();
  }

  // If you want to know specifically why 'true' or 'false' was
  // returned, pass the address of a Result variable as the last
  // parameter.
  bool solve(
      std::array<T, n> const& q,
      std::array<std::array<T, n>, n> const& M,
      std::array<T, n>& w,
      std::array<T, n>& z,
      typename LCPSolverShared<T>::Result* result = nullptr) {
    return LCPSolverShared<T>::solve(
        q.data(), M.front().data(), w.data(), z.data(), result);
  }

private:
  std::array<typename LCPSolverShared<T>::Variable, n + 1> m_array_var_basic;
  std::array<typename LCPSolverShared<T>::Variable, n + 1> m_array_var_nonbasic;
  std::array<T, 2 * (n + 1) * n> m_array_augmented;
  std::array<T, n + 1> m_array_q_min;
  std::array<T, n + 1> m_array_min_ratio;
  std::array<T, n + 1> m_array_ratio;
  std::array<T*, n> m_array_poly;
};

template <typename T>
class LCPSolver<T> : public LCPSolverShared<T> {
public:
  // Construction.  The member m_max_iterations is set by this call to the
  // default value n*n.
  LCPSolver(int32_t n) : LCPSolverShared<T>(n) {
    if (n > 0) {
      size_t np1 = static_cast<size_t>(n) + 1;
      m_vector_var_basic.resize(np1);
      m_vector_var_nonbasic.resize(np1);
      m_vector_augmented.resize(2 * np1 * n);
      m_vector_q_min.resize(np1);
      m_vector_min_ratio.resize(np1);
      m_vector_ratio.resize(np1);
      m_vector_poly.resize(n);

      this->m_var_basic    = m_vector_var_basic.data();
      this->m_var_nonbasic = m_vector_var_nonbasic.data();
      this->m_num_cols     = 2 * (n + 1);
      this->m_augmented    = m_vector_augmented.data();
      this->m_q_min        = m_vector_q_min.data();
      this->m_min_ratio    = m_vector_min_ratio.data();
      this->m_ratio        = m_vector_ratio.data();
      this->m_poly         = m_vector_poly.data();
    }
  }

  // The input q must have n elements and the input M must be an n-by-n
  // matrix stored in row-major order.  The outputs w and z have n
  // elements.  If you want to know specifically why 'true' or 'false'
  // was returned, pass the address of a Result variable as the last
  // parameter.
  bool solve(
      std::vector<T> const& q,
      std::vector<T> const& M,
      std::vector<T>& w,
      std::vector<T>& z,
      typename LCPSolverShared<T>::Result* result = nullptr) {
    if (this->m_dimension > static_cast<int32_t>(q.size())
        || this->m_dimension * this->m_dimension
               > static_cast<int32_t>(M.size())) {
      if (result) {
        *result = LCPSolverShared<T>::Result::INVALID_INPUT;
      }
      return false;
    }

    if (this->m_dimension > static_cast<int32_t>(w.size())) {
      w.resize(this->m_dimension);
    }

    if (this->m_dimension > static_cast<int32_t>(z.size())) {
      z.resize(this->m_dimension);
    }

    return LCPSolverShared<T>::solve(
        q.data(), M.data(), w.data(), z.data(), result);
  }

private:
  std::vector<typename LCPSolverShared<T>::Variable> m_vector_var_basic;
  std::vector<typename LCPSolverShared<T>::Variable> m_vector_var_nonbasic;
  std::vector<T> m_vector_augmented;
  std::vector<T> m_vector_q_min;
  std::vector<T> m_vector_min_ratio;
  std::vector<T> m_vector_ratio;
  std::vector<T*> m_vector_poly;
};

} // namespace crobot::math

